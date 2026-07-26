/**
 * Source-code templates for freshly scaffolded ROS 2 nodes (Python + C++).
 *
 * Extracted verbatim from {@link RosWorkspace} so the ~500 lines of embedded
 * template text no longer sit in the middle of the workspace class. Pure
 * functions only — no VS Code / filesystem dependencies.
 */
import {
    toPascalCase,
    escapePythonSingleQuotedString,
    escapeCppDoubleQuotedString,
} from '../utils/strings';

export type RosNodeTemplateKind =
    | 'none'
    | 'publisher'
    | 'subscriber'
    | 'service'
    | 'client'
    | 'timer';

/** Coerce arbitrary user input into a supported template kind. */
export function normalizeNodeTemplateKind(rawKind?: string): RosNodeTemplateKind {
    const normalized = String(rawKind || 'none').trim().toLowerCase();
    switch (normalized) {
        case 'publisher':
        case 'subscriber':
        case 'service':
        case 'client':
        case 'timer':
            return normalized;
        default:
            return 'none';
    }
}

/** Normalise a topic name, falling back to `chatter` and stripping whitespace. */
export function normalizeNodeTemplateTopic(rawTopic?: string): string {
    const trimmed = String(rawTopic || '').trim();
    if (!trimmed) {
        return 'chatter';
    }
    return trimmed.replace(/\s+/g, '_');
}

/** ament build dependencies implied by a given C++ template kind. */
export function getCppTemplateDependencies(templateKind: RosNodeTemplateKind): string[] {
    if (templateKind === 'publisher' || templateKind === 'subscriber') {
        return ['std_msgs'];
    }
    if (templateKind === 'service' || templateKind === 'client') {
        return ['example_interfaces'];
    }
    return [];
}

export function buildPythonNodeTemplate(
    safeNodeName: string,
    templateKind: RosNodeTemplateKind,
    templateTopic: string,
): string {
    const className = `${toPascalCase(safeNodeName)}Node`;
    const escapedTopicName = escapePythonSingleQuotedString(templateTopic);

    if (templateKind === 'publisher') {
        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # 1) Publisher: sends String messages to this topic.
        self.publisher_ = self.create_publisher(String, '${escapedTopicName}', 10)
        # 2) Timer: calls timer_callback every 0.5s.
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.counter_ = 0
        self.get_logger().info('${safeNodeName} publisher node started on topic ${escapedTopicName}')

    def timer_callback(self):
        # This function runs repeatedly from the timer.
        msg = String()
        msg.data = f'Hello from ${safeNodeName}: {self.counter_}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter_ += 1


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so callbacks continue running.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
    }

    if (templateKind === 'subscriber') {
        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create subscriber for String messages on this topic.
        self.subscription = self.create_subscription(
            String,
            '${escapedTopicName}',
            self.listener_callback,
            10,
        )
        self.get_logger().info('${safeNodeName} subscriber node listening on ${escapedTopicName}')

    def listener_callback(self, msg):
        # Called each time a new message is received.
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so callbacks continue running.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
    }

    if (templateKind === 'service') {
        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create a very simple service server.
        # Rename "add_two_ints" later if you want.
        self.service_ = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service ready: add_two_ints')

    def add_two_ints_callback(self, request, response):
        # request has "a" and "b"; fill response.sum.
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        return response


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Keep node alive so it can answer service calls.
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
    }

    if (templateKind === 'client') {
        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ${className}(Node):
    def __init__(self):
        # Give this node a unique runtime name.
        super().__init__('${safeNodeName}')
        # Create a client for the "add_two_ints" service.
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request_ = AddTwoInts.Request()
        self.get_logger().info('Client ready for service: add_two_ints')

    def send_request(self, a, b):
        # Fill request and send it asynchronously.
        self.request_.a = a
        self.request_.b = b
        return self.client_.call_async(self.request_)


def main(args=None):
    # rclpy setup.
    rclpy.init(args=args)
    node = ${className}()
    # Change these numbers to test different requests.
    future = node.send_request(2, 3)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f'Result: {future.result().sum}')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
    }

    if (templateKind === 'timer') {
        return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${className}(Node):
    def __init__(self):
        super().__init__('${safeNodeName}')
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.on_timer)
        self.get_logger().info('${safeNodeName} timer node started')

    def on_timer(self):
        self.counter_ += 1
        self.get_logger().info(f'Tick {self.counter_}')


def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
    }

    return `#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ${className}(Node):
    def __init__(self):
        super().__init__('${safeNodeName}')
        self.get_logger().info('${safeNodeName} node started')


def main(args=None):
    rclpy.init(args=args)
    node = ${className}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
`;
}

export function buildCppNodeTemplate(
    safeNodeName: string,
    templateKind: RosNodeTemplateKind,
    templateTopic: string,
): string {
    const className = `${toPascalCase(safeNodeName)}Node`;
    const escapedNodeName = escapeCppDoubleQuotedString(safeNodeName);
    const escapedTopicName = escapeCppDoubleQuotedString(templateTopic);

    if (templateKind === 'publisher') {
        return `#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}"), published_(false) {
    // 1) Publisher: sends String messages to this topic.
    publisher_ = this->create_publisher<std_msgs::msg::String>("${escapedTopicName}", 10);
    // 2) Timer: calls publish_once every 0.5s.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&${className}::publish_once, this)
    );
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} publisher node started on topic ${escapedTopicName}");
  }

private:
  void publish_once() {
    // This function runs repeatedly from the timer.
    if (published_) {
      return;
    }
    auto msg = std_msgs::msg::String();
    msg.data = "hello world my name is ${escapedNodeName}";
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    published_ = true;
    timer_->cancel();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool published_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
    }

    if (templateKind === 'subscriber') {
        return `#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    // Create subscriber for String messages on this topic.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "${escapedTopicName}",
      10,
      std::bind(&${className}::on_message, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} subscriber node listening on ${escapedTopicName}");
  }

private:
  void on_message(const std_msgs::msg::String::SharedPtr msg) const {
    // Called each time a new message is received.
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
    }

    if (templateKind === 'service') {
        return `#include <functional>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    // Create a very simple service server.
    // Rename "add_two_ints" later if you want.
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&${className}::add_two_ints, this, _1, _2)
    );
    RCLCPP_INFO(this->get_logger(), "Service ready: add_two_ints");
  }

private:
  void add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
  ) {
    // request has "a" and "b"; fill response->sum.
    response->sum = request->a + request->b;
    RCLCPP_INFO(
      this->get_logger(),
      "Incoming request: a=%ld, b=%ld",
      static_cast<long>(request->a),
      static_cast<long>(request->b)
    );
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
    }

    if (templateKind === 'client') {
        return `#include <chrono>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("${escapedNodeName}");

  // Create a client for the "add_two_ints" service.
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  while (!client->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 2;
  request->b = 3;

  // Send request and wait for result.
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(
      node->get_logger(),
      "Result: %ld",
      static_cast<long>(future.get()->sum)
    );
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
`;
    }

    return `#include <memory>

#include "rclcpp/rclcpp.hpp"

class ${className} : public rclcpp::Node {
public:
  ${className}()
  : Node("${escapedNodeName}") {
    RCLCPP_INFO(this->get_logger(), "${escapedNodeName} node started");
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${className}>());
  rclcpp::shutdown();
  return 0;
}
`;
}
