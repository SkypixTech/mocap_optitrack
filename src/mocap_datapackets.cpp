#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <ros/console.h>
using namespace std;

RigidBody::RigidBody() 
  : NumberOfMarkers(0), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

const geometry_msgs::PoseStamped RigidBody::get_ros_pose()
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  // y & z axes are swapped in the Optitrack coordinate system
  ros_pose.pose.position.x = pose.position.x;
  ros_pose.pose.position.y = -pose.position.z;
  ros_pose.pose.position.z = pose.position.y;

  ros_pose.pose.orientation.x = pose.orientation.x;
  ros_pose.pose.orientation.y = -pose.orientation.z;
  ros_pose.pose.orientation.z = pose.orientation.y;
  ros_pose.pose.orientation.w = pose.orientation.w;

  return ros_pose;
}

const geometry_msgs::PointStamped RigidBody::get_marker_1_pos()
{
  ROS_WARN("getting marker pos");
  geometry_msgs::PointStamped marker_pos;
  marker_pos.header.stamp = ros::Time::now();


  // read marker positions
  marker_pos.point.x = marker[0].positionX;
  marker_pos.point.y = marker[0].positionY;
  marker_pos.point.z = marker[0].positionZ;
  return marker_pos;

}


const geometry_msgs::PointStamped RigidBody::get_marker_2_pos()
{
  ROS_WARN("getting marker pos");
  geometry_msgs::PointStamped marker_pos;
  marker_pos.header.stamp = ros::Time::now();


  // read marker positions
  marker_pos.point.x = marker[1].positionX;
  marker_pos.point.y = marker[1].positionY;
  marker_pos.point.z = marker[1].positionZ;
  return marker_pos;

}


const geometry_msgs::PointStamped RigidBody::get_marker_3_pos()
{
  ROS_WARN("getting marker pos");
  geometry_msgs::PointStamped marker_pos;
  marker_pos.header.stamp = ros::Time::now();


  // read marker positions
  marker_pos.point.x = marker[2].positionX;
  marker_pos.point.y = marker[2].positionY;
  marker_pos.point.z = marker[2].positionZ;
  return marker_pos;

}


const geometry_msgs::PointStamped RigidBody::get_marker_4_pos()
{
  ROS_WARN("getting marker pos");
  geometry_msgs::PointStamped marker_pos;
  marker_pos.header.stamp = ros::Time::now();


  // read marker positions
  marker_pos.point.x = marker[3].positionX;
  marker_pos.point.y = marker[3].positionY;
  marker_pos.point.z = marker[3].positionZ;
  return marker_pos;

}

bool RigidBody::has_data()
{
  static const char zero[sizeof(pose)] = { 0 };
  return memcmp(zero, (char*) &pose, sizeof(pose));
}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markerSets(0), otherMarkers(0), rigidBodies(0),
    numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0),
    latency(0.0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length), frameNumber(0)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4);

  // parse frame number
  read_and_seek(frameNumber);

  // count number of packetsets
  read_and_seek(model.numMarkerSets);
  model.markerSets = new MarkerSet[model.numMarkerSets];
  ROS_INFO("Number of marker sets: %d\n", model.numMarkerSets);

  for (int i = 0; i < model.numMarkerSets; i++)
  {
    strcpy(model.markerSets[i].name, packet);
    seek(strlen(model.markerSets[i].name) + 1);

    //    ROS_INFO("Parsing marker set named: %s\n", model.markerSets[i].name);

    // read number of markers that belong to the model
    read_and_seek(model.markerSets[i].numMarkers);
    ROS_INFO("Number of markers in set: %d\n", model.markerSets[i].numMarkers);

    model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];
    for (int k = 0; k < model.markerSets[i].numMarkers; k++)
    {
      // read marker positions
      read_and_seek(model.markerSets[i].markers[k]);
    }
  }

  /*
   * publishing marker position
   */

  ROS_INFO("publishing marker position: %d\n", model.numMarkerSets);

  for (int i = 0; i < model.numMarkerSets; i++)
  {
    ROS_INFO("Parsing marker set named: %s\n", model.markerSets[i].name);

    ROS_INFO("Number of markers in set: %d\n", model.markerSets[i].numMarkers);


    for (int k = 0; k < model.markerSets[i].numMarkers; k++)
    {
      // read marker positions

      ROS_INFO("inteagle: position of marker %d : [%f , %f, %f] \n", k,
               model.markerSets[i].markers[k].positionX,
               model.markerSets[i].markers[k].positionY,
               model.markerSets[i].markers[k].positionZ);
    }
  }

  // read number of 'other' markers (cf. NatNet specs)
  read_and_seek(model.numOtherMarkers);
  model.otherMarkers = new Marker[model.numOtherMarkers];
  ROS_INFO("Number of markers not in sets: %d\n", model.numOtherMarkers);
  for (int l = 0; l < model.numOtherMarkers; l++)
  {
    // read positions of 'other' markers
    read_and_seek(model.otherMarkers[l]);
  }

  // read number of rigid bodies of the model
  read_and_seek(model.numRigidBodies);
  //  ROS_INFO("Number of rigid bodies: %d\n", model.numRigidBodies);

  model.rigidBodies = new RigidBody[model.numRigidBodies];




  ROS_INFO("----------------star of numMarkerSets------------ \n");
  ROS_INFO("model.numMarkerSets is: %d \n", model.numMarkerSets);
  ROS_INFO("model.markerSets[0].numMarkers is: %d \n", model.markerSets[0].numMarkers);


  for (int i = 0; i < model.markerSets[0].numMarkers; ++i)
  {
    //    ROS_INFO("inteagle: model.markerSets[0].numMarkers is %d  \n",
    //             model.markerSets[0].numMarkers); = 4

    ROS_INFO("inteagle: position of marker %d : [%f , %f, %f] \n", i,
             model.markerSets[0].markers[i].positionX,
        model.markerSets[0].markers[i].positionY,
        model.markerSets[0].markers[i].positionZ);
  }

  //  ROS_INFO("model.markerSets[0].numMarkers is: %d \n", model.markerSets[1].numMarkers);
  //  for (int i = 0; i < model.markerSets[1].numMarkers; ++i)
  //  {
  //    ROS_INFO("inteagle: model.markerSets[1].numMarkers is %d  \n",
  //             model.markerSets[1].numMarkers);

  //    ROS_INFO("inteagle: position of marker %d : [%f , %f, %f] \n", i,
  //             model.markerSets[1].markers[i].positionX,
  //        model.markerSets[1].markers[i].positionY,
  //        model.markerSets[1].markers[i].positionZ);
  //  }

  ROS_INFO("----------------end of numMarkerSets------------ \n");



  //  ROS_INFO("----------------start of numRigidBodies------------ \n");
  //  for (int i = 0; i < model.numRigidBodies; ++i)
  //  {
  ////    ROS_INFO("inteagle: model.numRigidBodies is %d  \n",
  ////             model.numRigidBodies);
  //    for(int j = 0; j< model.rigidBodies[i].NumberOfMarkers; ++j)
  //    {

  //      ROS_INFO("inteagle: position of marker %d of rigid body %d : [%f , %f, %f] \n",
  //               i, j,
  //               model.rigidBodies[i].marker[j].positionX,
  //               model.rigidBodies[i].marker[j].positionY,
  //               model.rigidBodies[i].marker[j].positionZ);
  //    }

  //  }

  //  ROS_INFO("----------------end of numRigidBodies------------ \n");

  ROS_INFO("----------------start of pose------------ \n");
  for (int i = 0; i < model.numRigidBodies; ++i)
  {
    geometry_msgs::PoseStamped ros_pose;
    ros_pose = model.rigidBodies[i].get_ros_pose();

    ROS_INFO("inteagle: position of rigid body %d: [%f , %f, %f] \n", i,
             ros_pose.pose.position.x,
             ros_pose.pose.position.y,
             ros_pose.pose.position.z);

    ROS_INFO("inteagle: position of rigid body %d: [%f , %f, %f] \n", i,
             model.rigidBodies[i].pose.position.x,
             model.rigidBodies[i].pose.position.y,
             model.rigidBodies[i].pose.position.z);


    //    ROS_INFO("inteagle: orientation of rigid body %d: [%f , %f, %f, %f] \n", i,
    //             ros_pose.pose.orientation.x,
    //             ros_pose.pose.orientation.y,
    //             ros_pose.pose.orientation.z,
    //             ros_pose.pose.orientation.w);

    //    ROS_INFO("inteagle: orientation of rigid body %d: [%f , %f, %f, %f] \n", i,
    //             model.rigidBodies[i].pose.orientation.x,
    //             model.rigidBodies[i].pose.orientation.y,
    //             model.rigidBodies[i].pose.orientation.z,
    //             model.rigidBodies[i].pose.orientation.w);

  }

  ROS_INFO("----------------end of pose------------ \n");



  for (int m = 0; m < model.numRigidBodies; m++)
  {
    // read id, position and orientation of each rigid body
    read_and_seek(model.rigidBodies[m].ID);
    read_and_seek(model.rigidBodies[m].pose);

    // get number of markers per rigid body
    read_and_seek(model.rigidBodies[m].NumberOfMarkers);
    ROS_DEBUG("Rigid body ID: %d\n", model.rigidBodies[m].ID);
    ROS_DEBUG("Number of rigid body markers: %d\n", model.rigidBodies[m].NumberOfMarkers);



    if (model.rigidBodies[m].NumberOfMarkers > 0)
    {
      model.rigidBodies[m].marker = new Marker [model.rigidBodies[m].NumberOfMarkers];
      size_t byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(Marker);
      memcpy(model.rigidBodies[m].marker, packet, byte_count);
      seek(byte_count);

      // skip marker IDs
      byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(int);
      seek(byte_count);

      // skip marker sizes
      byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(float);
      seek(byte_count);
    }

    // skip mean marker error
    seek(sizeof(float));
    seek(2);
  }

  // TODO: read skeletons
  int numSkeletons = 0;
  read_and_seek(numSkeletons);

  // get latency
  read_and_seek(model.latency);


}


