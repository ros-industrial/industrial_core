/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "simple_message/simple_message.h"
#include "simple_message/byte_array.h"
#include "simple_message/shared_types.h"
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/socket/udp_client.h"
#include "simple_message/socket/udp_server.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/ping_message.h"
#include "simple_message/ping_handler.h"
#include "simple_message/messages/joint_message.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_manager.h"
#include "simple_message/simple_comms_fault_handler.h"
#include "simple_message/joint_traj_pt.h"
#include "simple_message/messages/joint_traj_pt_message.h"
#include "simple_message/typed_message.h"
#include "simple_message/joint_traj.h"

#include <gtest/gtest.h>
// Use pthread instead of boost::thread so we can cancel the TCP/UDP server
// threads 
//#include <boost/thread/thread.hpp>
#include <pthread.h>

using namespace industrial::simple_message;
using namespace industrial::byte_array;
using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::udp_socket;
using namespace industrial::udp_client;
using namespace industrial::udp_server;
using namespace industrial::tcp_socket;
using namespace industrial::tcp_client;
using namespace industrial::tcp_server;
using namespace industrial::ping_message;
using namespace industrial::ping_handler;
using namespace industrial::joint_data;
using namespace industrial::joint_message;
using namespace industrial::message_manager;
using namespace industrial::simple_comms_fault_handler;
using namespace industrial::joint_traj_pt;
using namespace industrial::joint_traj_pt_message;
using namespace industrial::typed_message;
using namespace industrial::joint_traj;

TEST(ByteArraySuite, init)
{

  const shared_int SIZE = 100;

  ByteArray bytes;
  shared_int TOO_BIG = bytes.getMaxBufferSize()+1;

  char bigBuffer[TOO_BIG];
  char buffer[SIZE];

  // Valid byte arrays
  EXPECT_TRUE(bytes.init(&buffer[0], SIZE));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);

  // Invalid init (too big)
  // Invalid buffers
  EXPECT_FALSE(bytes.init(&bigBuffer[0], TOO_BIG));
}

TEST(ByteArraySuite, loading)
{
  const shared_int SIZE = 100;
  char buffer[SIZE];

  ByteArray bytes;
  ByteArray empty;

  ASSERT_TRUE(bytes.init(&buffer[0], SIZE));

  shared_bool bIN = true, bOUT = false;
  shared_int iIN = 999, iOUT = 0;
  shared_real rIN = 9999.9999, rOUT = 0;

  // Boolean loading
  EXPECT_TRUE(bytes.load(bIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_bool));
  EXPECT_TRUE(bytes.unload(bOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(bOUT, bIN);

  // Integer loading
  EXPECT_TRUE(bytes.load(iIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_int));
  EXPECT_TRUE(bytes.unload(iOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(iOUT, iIN);

  // Real loading
  EXPECT_TRUE(bytes.load(rIN));
  EXPECT_EQ(bytes.getBufferSize(), SIZE+sizeof(shared_real));
  EXPECT_TRUE(bytes.unload(rOUT));
  EXPECT_EQ((shared_int)bytes.getBufferSize(), SIZE);
  EXPECT_EQ(rOUT, rIN);

  // Unloading a single member (down to an empty buffer size)
  EXPECT_TRUE(empty.load(bIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_bool));
  EXPECT_TRUE(empty.unload(bOUT));
  EXPECT_EQ((int)empty.getBufferSize(), 0);
  EXPECT_EQ(bOUT, bIN);

  // Loading two members (unloading the first) and then checking the value of the second
  rOUT = 0.0;
  iOUT = 0;
  EXPECT_TRUE(empty.load(rIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_real));
  EXPECT_TRUE(empty.load(iIN));
  EXPECT_EQ(empty.getBufferSize(), sizeof(shared_real)+sizeof(shared_int));
  EXPECT_TRUE(empty.unloadFront((void*)&rOUT, sizeof(shared_real)));
  EXPECT_EQ(rOUT, rIN);
  EXPECT_TRUE(empty.unload(iOUT));
  EXPECT_EQ((int)empty.getBufferSize(), 0);
  EXPECT_EQ(iOUT, iIN);
}

TEST(ByteArraySuite, copy)
{

  const shared_int SIZE = 100;
  char buffer[SIZE];

  // Copy
  ByteArray copyFrom;
  ByteArray copyTo;
  ByteArray tooBig;


  shared_int TOO_BIG = tooBig.getMaxBufferSize()-1;
  char bigBuffer[TOO_BIG];

  EXPECT_TRUE(copyFrom.init(&buffer[0], SIZE));
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ((shared_int)copyTo.getBufferSize(), SIZE);
  EXPECT_TRUE(copyTo.load(copyFrom));
  EXPECT_EQ((shared_int)copyTo.getBufferSize(), 2*SIZE);

  // Copy too large
  EXPECT_TRUE(tooBig.init(&bigBuffer[0], TOO_BIG));
  EXPECT_FALSE(copyTo.load(tooBig));
  // A failed load should not change the buffer.
  EXPECT_EQ((shared_int)copyTo.getBufferSize(), 2*SIZE);
}

TEST(SimpleMessageSuite, init)
{
  SimpleMessage msg;
  ByteArray bytes;

  // Valid messages
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::TOPIC, ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::INVALID, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::SUCCESS, bytes));
  EXPECT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REPLY,ReplyTypes::FAILURE, bytes));

  // Unused command
  EXPECT_FALSE(msg.init(StandardMsgTypes::INVALID, CommTypes::INVALID,ReplyTypes::INVALID, bytes));

  // Service request with a reply
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::SUCCESS, bytes));
  EXPECT_FALSE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST,ReplyTypes::FAILURE, bytes));
}

TEST(PingMessageSuite, init)
{
  PingMessage ping;
  SimpleMessage msg;

  EXPECT_FALSE(ping.init(msg));
  ping.init();
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMessageType());

  ping = PingMessage();
  ASSERT_TRUE(msg.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID));
  EXPECT_TRUE(ping.init(msg));
  EXPECT_EQ(StandardMsgTypes::PING, ping.getMessageType());
}

TEST(PingMessageSuite, toMessage)
{
  PingMessage ping;
  SimpleMessage msg;

  ping.init();

  ASSERT_TRUE(ping.toReply(msg, ReplyTypes::SUCCESS));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REPLY, msg.getCommType());
  EXPECT_EQ(ReplyTypes::SUCCESS, msg.getReplyCode());

  ASSERT_TRUE(ping.toRequest(msg));
  EXPECT_EQ(StandardMsgTypes::PING, msg.getMessageType());
  EXPECT_EQ(CommTypes::SERVICE_REQUEST, msg.getCommType());
  EXPECT_EQ(ReplyTypes::INVALID, msg.getReplyCode());

  EXPECT_FALSE(ping.toTopic(msg));

}

TEST(PingHandlerSuite, init)
{
  PingHandler handler;
  UdpClient udp;

  ASSERT_TRUE(handler.init(&udp));
  EXPECT_EQ(StandardMsgTypes::PING, handler.getMsgType());

  EXPECT_FALSE(handler.init(NULL));

}

TEST(MessageManagerSuite, init)
{
  MessageManager manager;
  UdpClient udp;

  EXPECT_TRUE(manager.init(&udp));
  EXPECT_FALSE(manager.init(NULL));

}

TEST(MessageManagerSuite, addHandler)
{
  MessageManager manager;
  UdpClient udp;
  PingHandler handler;

  EXPECT_EQ(0, (int)manager.getNumHandlers());

  ASSERT_TRUE(manager.init(&udp));
  EXPECT_EQ(1, (int)manager.getNumHandlers());
  EXPECT_FALSE(manager.add(NULL));

  ASSERT_TRUE(handler.init(&udp));
  EXPECT_FALSE(manager.add(&handler));
}

// wrapper around MessageManager::spin() that can be passed to
// pthread_create()
void*
spinFunc(void* arg)
{
  MessageManager* mgr = (MessageManager*)arg;
  mgr->spin();
  return NULL;
}

TEST(DISABLED_MessageManagerSuite, udp)
{
  const int udpPort = 11000;
  char ipAddr[] = "127.0.0.1";

  UdpClient* udpClient = new UdpClient();
  UdpServer udpServer;
  SimpleMessage pingRequest, pingReply;
  MessageManager udpManager;

  ASSERT_TRUE(pingRequest.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID));

  // UDP Socket testing
  // Construct server and start in a thread
  ASSERT_TRUE(udpServer.init(udpPort));
  ASSERT_TRUE(udpManager.init(&udpServer));
  //boost::thread udpSrvThrd(boost::bind(&MessageManager::spin, &udpManager));
  pthread_t udpSrvThrd;
  pthread_create(&udpSrvThrd, NULL, spinFunc, &udpManager);

  // Construct a client and try to ping the server
  ASSERT_TRUE(udpClient->init(&ipAddr[0], udpPort));
  ASSERT_TRUE(udpClient->makeConnect());
  ASSERT_TRUE(udpClient->sendMsg(pingRequest));
  ASSERT_TRUE(udpClient->receiveMsg(pingReply));
  ASSERT_TRUE(udpClient->sendAndReceiveMsg(pingRequest, pingReply));

  // Delete client and try to reconnect
  delete udpClient;
  udpClient = new UdpClient();
  ASSERT_TRUE(udpClient->init(&ipAddr[0], udpPort));
  ASSERT_TRUE(udpClient->makeConnect());
  ASSERT_TRUE(udpClient->sendAndReceiveMsg(pingRequest, pingReply));

  pthread_cancel(udpSrvThrd);
  pthread_join(udpSrvThrd, NULL);
}

TEST(MessageManagerSuite, tcp)
{
  const int tcpPort = 11000;
  char ipAddr[] = "127.0.0.1";

  TcpClient* tcpClient = new TcpClient();
  TcpServer tcpServer;
  SimpleMessage pingRequest, pingReply;
  MessageManager tcpManager;

  // MessageManager uses ros::ok, which needs ros spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ASSERT_TRUE(pingRequest.init(StandardMsgTypes::PING, CommTypes::SERVICE_REQUEST, ReplyTypes::INVALID));

  // TCP Socket testing

  // Construct server
  ASSERT_TRUE(tcpServer.init(tcpPort));

  // Construct a client
  ASSERT_TRUE(tcpClient->init(&ipAddr[0], tcpPort));
  ASSERT_TRUE(tcpClient->makeConnect());

  // Listen for client connection, init manager and start thread
  ASSERT_TRUE(tcpServer.makeConnect());
  ASSERT_TRUE(tcpManager.init(&tcpServer));

  // TODO: The message manager is not thread safe (threads are used for testing,
  // but running the message manager in a thread results in errors when the
  // underlying connection is deconstructed before the manager
  //boost::thread tcpSrvThrd(boost::bind(&MessageManager::spin, &tcpManager));
  pthread_t tcpSrvThrd;
  pthread_create(&tcpSrvThrd, NULL, spinFunc, &tcpManager);

  // Ping the server
  ASSERT_TRUE(tcpClient->sendMsg(pingRequest));
  ASSERT_TRUE(tcpClient->receiveMsg(pingReply));
  ASSERT_TRUE(tcpClient->sendAndReceiveMsg(pingRequest, pingReply));

  // Delete client and try to reconnect

  delete tcpClient;
  sleep(10); //Allow time for client to destruct and free up port
  tcpClient = new TcpClient();

  ASSERT_TRUE(tcpClient->init(&ipAddr[0], tcpPort));
  ASSERT_TRUE(tcpClient->makeConnect());
  ASSERT_TRUE(tcpClient->sendAndReceiveMsg(pingRequest, pingReply));

  pthread_cancel(tcpSrvThrd);
  pthread_join(tcpSrvThrd, NULL);

  delete tcpClient;
}

TEST(JointMessage, init)
{
  JointData joint;

  joint.init();
  EXPECT_TRUE(joint.setJoint(0, 1.0));
  EXPECT_TRUE(joint.setJoint(1, 2.0));
  EXPECT_TRUE(joint.setJoint(2, 3.0));
  EXPECT_TRUE(joint.setJoint(3, 4.0));
  EXPECT_TRUE(joint.setJoint(4, 5.0));
  EXPECT_TRUE(joint.setJoint(5, 6.0));
  EXPECT_TRUE(joint.setJoint(6, 7.0));
  EXPECT_TRUE(joint.setJoint(7, 8.0));
  EXPECT_TRUE(joint.setJoint(8, 9.0));
  EXPECT_TRUE(joint.setJoint(9, 10.0));

  EXPECT_FALSE(joint.setJoint(10, 11.0));

}

TEST(JointMessage, equal)
{
  JointData jointlhs, jointrhs;

  jointrhs.init();
  jointlhs.init();
  jointlhs.setJoint(0, -1.0);
  jointlhs.setJoint(9, 1.0);

  EXPECT_FALSE(jointlhs==jointrhs);

  jointrhs.setJoint(0, -1.0);
  jointrhs.setJoint(9, 1.0);

  EXPECT_TRUE(jointlhs==jointrhs);

}

TEST(JointMessage, toMessage)
{
  JointData toMessage, fromMessage;
  JointMessage msg;

  toMessage.init();
  toMessage.setJoint(4, 44.44);

  msg.init(1, toMessage);

  fromMessage.copyFrom(msg.getJoints());

  EXPECT_TRUE(toMessage==fromMessage);

}

// Message passing routine, used to send and receive a typed message
// Useful for checking the packing and unpacking of message data.
void messagePassing(TypedMessage &send, TypedMessage &recv)
{
	const int tcpPort = 11010;
	  char ipAddr[] = "127.0.0.1";

	  TcpClient tcpClient;
	  TcpServer tcpServer;
	  SimpleMessage msgSend, msgRecv;

	  ASSERT_TRUE(send.toTopic(msgSend));

	  // Construct server

	  ASSERT_TRUE(tcpServer.init(tcpPort));

	  // Construct a client
	  ASSERT_TRUE(tcpClient.init(&ipAddr[0], tcpPort));
	  ASSERT_TRUE(tcpClient.makeConnect());

	  ASSERT_TRUE(tcpServer.makeConnect());

	  ASSERT_TRUE(tcpClient.sendMsg(msgSend));
	  ASSERT_TRUE(tcpServer.receiveMsg(msgRecv));
	  ASSERT_TRUE(recv.init(msgRecv));
}

TEST(JointMessage, Comms)
{

  JointMessage jointSend, jointRecv;
  JointData posSend, posRecv;

  posSend.init();
  posSend.setJoint(0,1.0);
  posSend.setJoint(1,2.0);
  posSend.setJoint(2,3.0);
  posSend.setJoint(3,4.0);
  posSend.setJoint(4,5.0);
  posSend.setJoint(5,6.0);
  posSend.setJoint(6,7.0);
  posSend.setJoint(7,8.0);
  posSend.setJoint(8,9.0);
  posSend.setJoint(9,10.0);

  jointSend.init(1, posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.getJoints());
  ASSERT_TRUE(posRecv==posSend);
}


TEST(JointTrajPt, equal)
{
	JointTrajPt lhs, rhs;
	JointData joint;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  rhs.init(1.0, joint, 50.0);
  EXPECT_FALSE(lhs==rhs);

  lhs.init(0, joint, 0);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}

TEST(JointTrajPt, toMessage)
{
	JointTrajPt toMessage, fromMessage;
	JointTrajPtMessage msg;

  toMessage.init();
  toMessage.setSequence(99);
  msg.init(toMessage);

  fromMessage.copyFrom(msg.point_);

  EXPECT_TRUE(toMessage==fromMessage);

}


TEST(JointTrajPt, Comms)
{

	JointTrajPtMessage jointSend, jointRecv;
	JointData data;
	JointTrajPt posSend, posRecv;

	data.init();
	data.setJoint(0,1.0);
	data.setJoint(1,2.0);
	data.setJoint(2,3.0);
	data.setJoint(3,4.0);
	data.setJoint(4,5.0);
	data.setJoint(5,6.0);
	data.setJoint(6,7.0);
	data.setJoint(7,8.0);
	data.setJoint(8,9.0);
	data.setJoint(9,10.0);
	posSend.init(1, data, 99);

  jointSend.init(posSend);

  messagePassing(jointSend, jointRecv);

  posRecv.copyFrom(jointRecv.point_);
  ASSERT_TRUE(posRecv==posSend);
}

TEST(JointTraj, equal)
{
	JointTraj lhs, rhs;
	JointData joint;
	JointTrajPt point;

  joint.init();
  ASSERT_TRUE(joint.setJoint(0, 1.0));
  ASSERT_TRUE(joint.setJoint(1, 2.0));
  ASSERT_TRUE(joint.setJoint(2, 3.0));
  ASSERT_TRUE(joint.setJoint(3, 4.0));
  ASSERT_TRUE(joint.setJoint(4, 5.0));
  ASSERT_TRUE(joint.setJoint(5, 6.0));
  ASSERT_TRUE(joint.setJoint(6, 7.0));
  ASSERT_TRUE(joint.setJoint(7, 8.0));
  ASSERT_TRUE(joint.setJoint(8, 9.0));
  ASSERT_TRUE(joint.setJoint(9, 10.0));

  point.init(1.0, joint, 50.0);
  rhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_TRUE(lhs==rhs);

  lhs.addPoint(point);
  EXPECT_FALSE(lhs==rhs);

  lhs.copyFrom(rhs);
  EXPECT_TRUE(lhs==rhs);

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

