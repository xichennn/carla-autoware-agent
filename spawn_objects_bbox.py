#!/usr/bin/env python

import os
import sys
import carla
import random
import rospy
from carla_msgs.msg import CarlaWorldInfo
from carla import VehicleLightState as vls

import logging
logging.basicConfig()

#bbox
import cv2
from sensor_msgs.msg import Image
import utils
from cv_bridge import CvBridge

class SpawnObjects(object):

    """
    Handles the spawning of objects
    """

    def __init__(self):
        self.host = rospy.get_param('/carla/host', 'localhost')
        self.port = rospy.get_param('/carla/port', 2000)
        self.timeout = rospy.get_param('/carla/timeout', 10)
        #self.client = None
        self.client = carla.Client(self.host, self.port)
        self.world = None
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.player = None
        self.player_created = False
        self.actor_list= []
        self.actor_spawnpoint = None
        self.number_of_vehicles = 100
        self.car_lights_on = False
        self.tm_port = 8000
        self.tm_seed = 2021
        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.synchronous_master = False
        self.vehicles_list = []
        
        #walkers parameters
        self.percentagePedestriansRunning = 10.0
        self.percentagePedestriansCrossing = 20.0
        self.number_of_walkers = 100
        self.walkers_list = []
        self.all_id = []
        self.all_actors = []
        
        #bbox
        self.img_pub_msg = None
 
    def draw_bbox_callback(self,msg):
        actor_list = self.world.get_actors()
        vehicle_list = actor_list.filter('vehicle.*')
        rgb_cameras = actor_list.filter('sensor.camera.rgb')
        for rgb_camera in rgb_cameras:
            if rgb_camera.attributes['role_name'] == 'front':
               cam = rgb_camera

        box2d = utils.auto_annotate(vehicle_list,cam,depth_img=None)
#        rospy.loginfo("There are {}, boxes are: {}".format(len(box2d),box2d))
        #img_pub = rospy.Publisher('/image_rects_bbox',Image, queue_size=10)

        if box2d:
            if msg is not None:
                rospy.loginfo("showing images with 2D bounding box...")
                img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
            for bbox in box2d:
                img = cv2.rectangle(img, (int(bbox[0][0]), int(bbox[0][1])),
                                         (int(bbox[1][0]), int(bbox[1][1])),(0,0,255),2)
                #img_pub.publish(CvBridge().cv2_to_imgmsg(img_bbox,'bgr8'))
            self.img_pub_msg = CvBridge().cv2_to_imgmsg(img,'bgr8')
            #cv2.imshow('box2d',img_bbox)
            #cv2.imshow('image',img_bbox)



    def destroy(self):
        """
        destroy all the actors
        """
        rospy.loginfo('destroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_id),2):
            self.all_actors[i].stop()

        rospy.loginfo('destroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])

        #rate.sleep(0.5)

        #if self.player and self.player.is_alive:
        #   self.player.destroy()
        #self.player = None

    def spawn_at_random_location_walkers(self):
        blueprintWalkers = self.world.get_blueprint_library().filter('walker.pedestrian.*')
        #1. take all the random locations to spawn
        spawn_points = []

        for i in range(self.number_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        #2. we spawn the walker object
        batch = []
        walker_speed = []

        SpawnActor = carla.command.SpawnActor

        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintWalkers)
            #set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            #set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > self.percentagePedestriansRunning):
                    #walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    #running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                rospy.loginfo('Walker has no speed')
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch,True)
        
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                rospy.logerr(results[i].error)
            else:
                self.walkers_list.append({'id':results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        #3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
         
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                rospy.logerr(results[i].error)

            else:
                self.walkers_list[i]["con"] = results[i].actor_id

        #4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]['con'])
            self.all_id.append(self.walkers_list[i]['id'])
        self.all_actors = self.world.get_actors(self.all_id)
        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if not self.synchronous_master:
            self.world.wait_for_tick()
        else:
            self.world.tick()

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.set_pedestrians_cross_factor(self.percentagePedestriansCrossing)
        for i in range(0, len(self.all_id), 2):
            # start walker
            self.all_actors[i].start()
            # set walk to random point
            self.all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            self.all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
        #rospy.loginfo('spawned {0} vehicles and {1} walkers, press Ctrl+C to exit.'.format(len(self.vehicles_list), len(self.walkers_list))))

        # example of how to use parameters
        self.traffic_manager.global_percentage_speed_difference(30.0)

#        while True:
#           if  self.synchronous_master:
#                self.world.tick()
#            else:
#                self.world.wait_for_tick()


     
    def spawn_at_random_location_vehicles(self):
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        #traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        #traffic_manager.set_hybrid_physics_mode(True)
        self.traffic_manager.set_random_device_seed(self.tm_seed)


        if self.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif self.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            rospy.logwarn(msg, self.number_of_vehicles, number_of_spawn_points)
            self.number_of_vehicles = number_of_spawn_points

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor
        
        #spawn vehicles#
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= self.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            
            #prepare the light state of the cars to spawn
            light_state = vls.NONE
            if self.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            #spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, True, self.traffic_manager.get_port()))
                    .then(SetVehicleLightState(FutureActor, light_state)))
        
        for response in self.client.apply_batch_sync(batch,self.synchronous_master):
            if response.error:
                rospy.logerr(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

    def spawn_static_objects(self):
        self.actor_list = self.world.get_actors()
        vehicles = self.actor_list.filter('vehicle.*')
        blueprint_library = self.world.get_blueprint_library()
        for vehicle in vehicles:
            if vehicle.attributes['role_name'] == self.role_name:
                self.player = vehicle
        ego_location = self.player.get_transform()
        ego_location.location.y -=50
        traffic_cone = self.world.try_spawn_actor(
                            random.choice(blueprint_library.filter('static.prop.trafficcone01')), ego_location)
        ego_location.location.y +=50



    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Spawn objects Waiting for CARLA world (topic: /carla/world_info)...")
        #try:
        #    rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        #except rospy.ROSException:
        #    rospy.logerr("Timeout while waiting for world info!")
        #    sys.exit(1)

        #rospy.loginfo("CARLA world available. Spawn npc objects...")

        #self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()
	
	#Set weather parameters
	self.world.set_weather(carla.WeatherParameters.ClearNoon)
        settings =self.world.get_settings()
        settings.synchronous_mode = True
        
        #spawn objects
        self.spawn_static_objects()
        self.spawn_at_random_location_vehicles()
        self.spawn_at_random_location_walkers()

        #bounding box for vehicles in front camera fov
        self.image_pub = rospy.Publisher('/image_rects_bbox',Image, queue_size=10)
        self.subscriber = rospy.Subscriber('/image_raw', Image, self.draw_bbox_callback)
        rospy.loginfo("subscribed to /image_raw")
        
#        try:
#            rospy.spin()
#        except rospy.ROSInterruptException:
#            pass


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    spawn_objects = SpawnObjects()
    try:   
        rospy.init_node('gt_bbox')
        rospy.loginfo('gt_bbox node started')
        spawn_objects.run()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
       # rospy.loginfo("publishing image with error %s" % rospy.get_time())
            if spawn_objects.img_pub_msg is not None:
                spawn_objects.image_pub.publish(spawn_objects.img_pub_msg)
            #rospy.loginfo("publishing image with error")
            rate.sleep()

    
    except rospy.ROSInterruptException:
        pass
    finally:
        if spawn_objects.all_id is not None:
            spawn_objects.destroy()
        del spawn_objects.world
        del spawn_objects.client


if __name__ == '__main__':
    main()
            	

