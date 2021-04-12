#include <iostream>
#include <cstdio>
#include <zmq.hpp>

#include <franka_lwi_communication_protocol.h>

void throttledPrintState(frankalwi::proto::StateMessage<7> state, int skip, double avg_freq) {
  static int count = 0;
  if (count > skip) {
    printf("Average frequency of state messages: % 3.3f\n", avg_freq);

    std::cout << "Joints --------------" << std::endl;
    printf("Joint positions: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.jointPosition[0],
           state.jointPosition[1],
           state.jointPosition[2],
           state.jointPosition[3],
           state.jointPosition[4],
           state.jointPosition[5],
           state.jointPosition[6]);

    std::cout << "Joints --------------" << std::endl;
    printf("Joint velocities: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.jointVelocity[0],
           state.jointVelocity[1],
           state.jointVelocity[2],
           state.jointVelocity[3],
           state.jointVelocity[4],
           state.jointVelocity[5],
           state.jointVelocity[6]);

    std::cout << "Joints --------------" << std::endl;
    printf("Joint torques: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.jointTorque[0],
           state.jointTorque[1],
           state.jointTorque[2],
           state.jointTorque[3],
           state.jointTorque[4],
           state.jointTorque[5],
           state.jointTorque[6]);

    std::cout << "STATE --------------" << std::endl;
    printf("State position xyz:     % 3.3f, % 3.3f, % 3.3f\n",
           state.eePose.position.x,
           state.eePose.position.y,
           state.eePose.position.z);
    printf("State orientation wxyz: % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.eePose.orientation.w,
           state.eePose.orientation.x,
           state.eePose.orientation.y,
           state.eePose.orientation.z);

    std::cout << "EE Twist --------------" << std::endl;
    printf("EE twist: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.eeTwist.linear.x,
           state.eeTwist.linear.y,
           state.eeTwist.linear.z,
           state.eeTwist.angular.x,
           state.eeTwist.angular.y,
           state.eeTwist.angular.z);

    std::cout << "EE Wrench --------------" << std::endl;
    printf("EE wrench: % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f, % 3.3f\n",
           state.eeWrench.linear.x,
           state.eeWrench.linear.y,
           state.eeWrench.linear.z,
           state.eeWrench.angular.x,
           state.eeWrench.angular.y,
           state.eeWrench.angular.z);

    const char map[3] = {'X', 'Y', 'Z'};
    for (std::size_t dof = 0; dof < 6; ++dof) {
      std::string space = dof < 3 ? "linear " : "angular";
      printf("Jacobian %s %c: ", space.c_str(), map[dof % 3]);
      for (std::size_t joint = 0; joint < 6; ++joint) {
        printf("% 5.2f, ", state.jacobian[dof + joint * 6]);
      }
      printf("% 5.2f\n", state.jacobian[dof + 6 * 6]);
    }

    for (std::size_t row = 0; row < 7; ++row) {
      printf("Inertial: ");
      for (std::size_t column = 0; column < 6; ++column) {
        printf("% 5.2f, ", state.mass[row + column * 7]);
      }
      printf("% 5.2f\n", state.mass[row + 6 * 7]);
    }
    count = 0;
  }
  ++count;
}

void zmq_configure(zmq::context_t& context,
                   zmq::socket_t& publisher,
                   zmq::socket_t& subscriber) {
  context = zmq::context_t(1);

  subscriber = zmq::socket_t(context, ZMQ_SUB);
  subscriber.set(zmq::sockopt::conflate, 1);
  subscriber.set(zmq::sockopt::subscribe, "");
  subscriber.bind("tcp://0.0.0.0:5550");

  publisher = zmq::socket_t(context, ZMQ_PUB);
  publisher.bind("tcp://0.0.0.0:5551");
}

bool zmq_receive(zmq::socket_t& subscriber,
                 frankalwi::proto::StateMessage<7>& obj,
                 const zmq::recv_flags flags = zmq::recv_flags::none) {
  zmq::message_t message;
  auto res = subscriber.recv(message, flags);
  if (res) {
    obj = *message.data<frankalwi::proto::StateMessage<7>> ();
  }
  return res.has_value();
}

bool zmq_send(zmq::socket_t& publisher,
              const frankalwi::proto::CommandMessage<7>& obj) {
  zmq::message_t message(sizeof(obj));
  memcpy(message.data(), &obj, sizeof(obj));
  auto res = publisher.send(message, zmq::send_flags::none);
  return res.has_value();
}

int main(int argc, char** argv) {

  std::cout << std::fixed << std::setprecision(3);

  // Set up ZMQ
  zmq::context_t context = zmq::context_t(1);
  zmq::socket_t publisher, subscriber;
  zmq_configure(context, publisher, subscriber);

  frankalwi::proto::StateMessage<7> state{};
  frankalwi::proto::CommandMessage<7> command{};

  auto start = std::chrono::system_clock::now();
  int iterations;
  bool received = false;
  while (subscriber.connected()) {
    // blocking receive until we get a state from the robot
    if (zmq_receive(subscriber, state)) {
      if (!received) {
        received = true;
        start = std::chrono::system_clock::now();
        iterations = 0;
      }
      std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
      throttledPrintState(state, 500, iterations / elapsed_seconds.count());

      // compute the desired command here
      command.jointTorque[0] = 0.01;
      command.jointTorque[1] = 0.02;
      command.jointTorque[2] = 0.03;
      command.jointTorque[3] = 0.04;
      command.jointTorque[4] = 0.05;
      command.jointTorque[5] = 0.06;
      command.jointTorque[6] = 0.07;
      zmq_send(publisher, command);
      ++iterations;
    }
  }
}

