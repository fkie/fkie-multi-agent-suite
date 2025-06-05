// ****************************************************************************
//
// Copyright(c) 2014 - 2024 Fraunhofer FKIE
// Author : Alexander Tiderko
// License : MIT
//
// ****************************************************************************

// This code is based on rmw_fastrtps/rmw_fastrtps_shared_cpp/include/rmw_fastrtps_shared_cpp/custom_participant_info.hpp

#include <chrono>
#include <map>
#include <mutex>
#include <regex>
#include <string>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <limits.h>
#include <csignal>

#include "builtin_interfaces/msg/duration.hpp"
#include "fkie_mas_msgs/msg/changed_state.hpp"
#include "fkie_mas_msgs/msg/gid.hpp"
#include "fkie_mas_msgs/msg/participant_entities_info.hpp"
#include "fkie_mas_msgs/msg/participants.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

#include <fastdds/rtps/RTPSDomain.hpp>
#include <fastdds/rtps/participant/RTPSParticipantListener.hpp>

using namespace std::chrono_literals;
using namespace fkie_mas_msgs::msg;

using participant_map_t = std::map<eprosima::fastdds::rtps::GUID_t, ParticipantEntitiesInfo>;


// from rmw_fastrtps_shared_cpp
template <typename ByteT>
void copy_from_byte_array_to_fastrtps_guid(
    const ByteT *guid_byte_array,
    eprosima::fastdds::rtps::GUID_t *guid)
{
    static_assert(
        std::is_same<uint8_t, ByteT>::value || std::is_same<int8_t, ByteT>::value,
        "ByteT should be either int8_t or uint8_t");
    assert(guid_byte_array);
    assert(guid);
    constexpr auto prefix_size = sizeof(guid->guidPrefix.value);
    memcpy(guid->guidPrefix.value, guid_byte_array, prefix_size);
    memcpy(guid->entityId.value, &guid_byte_array[prefix_size], guid->entityId.size);
}

// from rmw_fastrtps_shared_cpp
template <typename ByteT>
void copy_from_fastrtps_guid_to_byte_array(
    const eprosima::fastdds::rtps::GUID_t &guid,
    ByteT *guid_byte_array)
{
    static_assert(
        std::is_same<uint8_t, ByteT>::value || std::is_same<int8_t, ByteT>::value,
        "ByteT should be either int8_t or uint8_t");
    assert(guid_byte_array);
    constexpr auto prefix_size = sizeof(guid.guidPrefix.value);
    memcpy(guid_byte_array, &guid.guidPrefix, prefix_size);
    memcpy(&guid_byte_array[prefix_size], &guid.entityId, guid.entityId.size);
}

// FROM rmw::impl::cpp
std::map<std::string, std::vector<uint8_t>>
parse_key_value(const std::vector<uint8_t> & kv)
{
  std::map<std::string, std::vector<uint8_t>> m;

  bool keyfound = false;

  std::string key;
  std::vector<uint8_t> value;
  uint8_t prev = '\0';

  if (kv.size() == 0) {
    goto not_valid;
  }

  for (uint8_t u8 : kv) {
    if (keyfound) {
      if ((u8 == ';') && (prev != ';')) {
        prev = u8;
        continue;
      } else if ((u8 != ';') && (prev == ';')) {
        if (value.size() == 0) {
          goto not_valid;
        }
        m[key] = value;

        key.clear();
        value.clear();
        keyfound = false;
      } else {
        value.push_back(u8);
      }
    }
    if (!keyfound) {
      if (u8 == '=') {
        if (key.size() == 0) {
          goto not_valid;
        }
        keyfound = true;
      } else if (isalnum(u8)) {
        key.push_back(u8);
      } else if ((u8 == '\0') && (key.size() == 0) && (m.size() > 0)) {
        break;  // accept trailing '\0' characters
      } else if ((prev != ';') || (key.size() > 0)) {
        goto not_valid;
      }
    }
    prev = u8;
  }
  if (keyfound) {
    if (value.size() == 0) {
      goto not_valid;
    }
    m[key] = value;
  } else if (key.size() > 0) {
    goto not_valid;
  }
  return m;
not_valid:
  // This is not a failure this is something that can happen because the participant_qos userData
  // is used. Other participants in the system not created by rmw could use userData for something
  // else.
  return std::map<std::string, std::vector<uint8_t>>();
}

/**
 * Get a value from a environment variable
 */
std::string getEnvironmentVariable(std::string const &key)
{
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
}

void convert_gid_to_msg(const eprosima::fastdds::rtps::GUID_t &gid, fkie_mas_msgs::msg::Gid &msg_gid)
{
    copy_from_fastrtps_guid_to_byte_array(gid, const_cast<uint8_t *>(msg_gid.data.begin()));
}

void convert_msg_to_gid(const fkie_mas_msgs::msg::Gid &msg_gid, eprosima::fastdds::rtps::GUID_t &gid)
{
    copy_from_byte_array_to_fastrtps_guid(const_cast<uint8_t *>(msg_gid.data.begin()), &gid);
}

class CustomParticipantListener : public rclcpp::Node,
                                  public eprosima::fastdds::rtps::RTPSParticipantListener
{
public:
    CustomParticipantListener(const std::string &node_name, const std::string &namespace_) : Node(node_name, namespace_),
                                                                                             eprosima::fastdds::rtps::RTPSParticipantListener()
    {
        on_shutdown = false;
        RCLCPP_INFO(get_logger(), "Node name: %s", node_name.c_str());
        publisher_ = this->create_publisher<fkie_mas_msgs::msg::ChangedState>("~/changed", rclcpp::QoS(10));
        RCLCPP_INFO(get_logger(), "  publisher: %s [fkie_mas_msgs/msg/ChangedState]", this->publisher_->get_topic_name());
        publisher_participants_ = this->create_publisher<fkie_mas_msgs::msg::Participants>("~/participants", rclcpp::QoS(1).reliable().transient_local().keep_last(1));
        RCLCPP_INFO(get_logger(), "  publisher: %s [fkie_mas_msgs/msg/Participants]", this->publisher_participants_->get_topic_name());
        daemon_topic_ = std::string("rt") + this->publisher_->get_topic_name();
        participant_ = nullptr;
        eprosima::fastdds::rtps::RTPSParticipantAttributes participant_attr;
        std::string full_name = this->get_node_base_interface()->get_fully_qualified_name();
        RCLCPP_INFO(get_logger(), "create eProsima participant: %s", full_name.c_str());
        participant_attr.setName(full_name.erase(0, 1).c_str());
        char *ros_domain_id = getenv("ROS_DOMAIN_ID");
        uint32_t domain_id = 0;
        if (ros_domain_id != NULL)
        {
            RCLCPP_INFO(get_logger(), "listen to domain id: %s", ros_domain_id);
            domain_id = atoi(ros_domain_id);
        }
        participant_ = eprosima::fastdds::rtps::RTPSDomain::createParticipant(domain_id, participant_attr, this);
    }

    void shutdown()
    {
        this->on_shutdown = true;
        eprosima::fastdds::rtps::RTPSDomain::stopAll();
    }

    void on_reader_discovery(
            eprosima::fastdds::rtps::RTPSParticipant* participant,
            eprosima::fastdds::rtps::ReaderDiscoveryStatus reason,
            const eprosima::fastdds::rtps::SubscriptionBuiltinTopicData& info,
            bool& /* should_be_ignored */)
    {
        static_cast<void>(participant);
        static_cast<void>(reason);
        static_cast<void>(info);

        // if (eprosima::fastdds::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastdds::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        //     {
        //         isnew = true;
        //     }
        //     else if (info.status == eprosima::fastdds::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        //     {
        //     }
        //     process_discovery_info(info.info, isnew, fkie_mas_msgs::msg::TopicEntity::INFO_READER);
        // }
        std::string name = info.topic_name.c_str();
        publish_notification(2, name);
        // if (name.rfind("rq/", 0) != 0 && name.rfind("rr/", 0) != 0)
        // {
        // }
    }

    void on_writer_discovery(
            eprosima::fastdds::rtps::RTPSParticipant* participant,
            eprosima::fastdds::rtps::WriterDiscoveryStatus reason,
            const eprosima::fastdds::rtps::PublicationBuiltinTopicData& info,
            bool& /* should_be_ignored */)
    {
        static_cast<void>(participant);
        static_cast<void>(reason);
        static_cast<void>(info);

        // if (eprosima::fastdds::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastdds::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        //     {
        //         isnew = true;
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher discovered");
        //     }
        //     else if (info.status == eprosima::fastdds::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        //     {
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher removed");
        //     }
        //     process_discovery_info(info.info, isnew, fkie_mas_msgs::msg::TopicEntity::INFO_WRITER);
        // }
        std::string name = info.topic_name.c_str();
        publish_notification(1, name);
        // if (name.rfind("rq/", 0) != 0 && name.rfind("rr/", 0) != 0)
        // {
        //     publish_notification(1, name);
        // }
    }

    template <class T>
    std::string to_string(T &data)
    {
        std::stringstream stream;
        stream << data;
        return stream.str();
    }

    void publish_notification(uint8_t type, std::string name = std::string(""))
    {
        std::lock_guard<std::mutex> guard(mutex_);
        fkie_mas_msgs::msg::ChangedState msg;
        msg.type = type;
        msg.topic_name = name;
        RCLCPP_DEBUG(get_logger(), "rosgraph: publish change notification of type %i with topic_name: %s", msg.type, msg.topic_name.c_str());
        publisher_->publish(msg);
    }

    // not mutex locked!!!
    void publish_participants()
    {
        fkie_mas_msgs::msg::Participants msg;
        for (auto it_dp = discoveredParticipants_.begin(); it_dp != discoveredParticipants_.end(); it_dp++)
        {
            msg.participants.push_back(it_dp->second);
        }
        RCLCPP_INFO(get_logger(), " publish update with with %lu participants", msg.participants.size());
        publisher_participants_->publish(msg);
    }

    void on_participant_discovery(
            eprosima::fastdds::rtps::RTPSParticipant* participant,
            eprosima::fastdds::rtps::ParticipantDiscoveryStatus reason,
            const eprosima::fastdds::rtps::ParticipantBuiltinTopicData& info,
            bool& /* should_be_ignored */)
    {
        static_cast<void>(participant);
        static_cast<void>(reason);
        static_cast<void>(info);

        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery: participant info %s", info.participant_name.c_str());
        auto participant_guid = to_string(info.guid);
        switch (reason)
        {
        case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DISCOVERED_PARTICIPANT:
        {
            RCLCPP_INFO(get_logger(), "onParticipantDiscovery: new participant %s", participant_guid.c_str());
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.guid);
            if (itp == discoveredParticipants_.end())
            {
                ParticipantEntitiesInfo pei;
                fkie_mas_msgs::msg::Gid gid;
                convert_gid_to_msg(info.guid, gid);
                pei.guid = gid;
                discoveredParticipants_[info.guid] = pei;
            }
            auto pi = discoveredParticipants_[info.guid];
            auto map = parse_key_value(info.user_data);
            // get as defined since foxy
            auto name_found = map.find("name");
            if (name_found != map.end())
            {
                pi.enclave = std::string(name_found->second.begin(), name_found->second.end());
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   name '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            } // get as defined since foxy
            auto enclave_found = map.find("enclave");
            if (enclave_found != map.end())
            {
                pi.enclave = std::string(enclave_found->second.begin(), enclave_found->second.end());
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery:   enclave '%s' found for %s", pi.enclave.c_str(), participant_guid.c_str());
            }
            for (auto i = info.default_locators.unicast.begin(); i != info.default_locators.unicast.end(); ++i)
            {
                RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:  add unicast locator: %s", to_string(*i).c_str());
                pi.unicast_locators.push_back(to_string(*i));
            }
            discoveredParticipants_[info.guid] = pi;
            publish_participants();
            break;
        }
        case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::REMOVED_PARTICIPANT:
        // fall through
        case eprosima::fastdds::rtps::ParticipantDiscoveryStatus::DROPPED_PARTICIPANT:
        {
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.guid);
            // only consider known GUIDs
            if (itp != discoveredParticipants_.end())
            {
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery: remove participant %s:", to_string(info.guid).c_str());
                discoveredParticipants_.erase(itp);
            }
            publish_participants();
            break;
        }
        default:
            return;
        }
        publish_notification(0);
    }

    /** Returns current time in seconds. */
    uint64_t now()
    {
        auto time = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
    }

private:
    mutable std::mutex mutex_;
    bool on_shutdown;
    eprosima::fastdds::rtps::RTPSParticipant *participant_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    rclcpp::Publisher<fkie_mas_msgs::msg::ChangedState>::SharedPtr publisher_;
    rclcpp::Publisher<fkie_mas_msgs::msg::Participants>::SharedPtr publisher_participants_;
    std::string daemon_topic_;
    double stamp_;
    participant_map_t discoveredParticipants_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

#include <iostream>

void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    rclcpp::init(argc, argv);
    char hostname_chars[HOST_NAME_MAX];
    gethostname(hostname_chars, HOST_NAME_MAX);
    // ROS_IP and ROS_HOSTNAME are optional environment variable that sets the declared network
    // address of a ROS Node or tool.The options are mutually exclusive, if both are set
    // ROS_HOSTNAME will take precedence
    char *ip_val = getenv("ROS_IP");
    std::string hostname(ip_val == NULL ? std::string(hostname_chars) : std::string(ip_val));
    char *host_val = getenv("ROS_HOSTNAME");
    hostname = host_val == NULL ? std::string(hostname) : std::string(host_val);
    // remove domain suffix
    std::regex const IP4_PATTERN{"^\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}\\.\\d{1,3}"};
    std::smatch m;
    if (!std::regex_match(hostname, m, IP4_PATTERN))
    {
        std::size_t found = hostname.find('.');
        if (found != std::string::npos)
        {
            hostname = hostname.substr(0, found);
        }
    }
    // replace dots and - characters in the node name
    hostname = std::regex_replace(hostname, std::regex("\\."), "_");
    hostname = std::regex_replace(hostname, std::regex("-"), "_");

    std::string node_name = "_discovery_" + hostname;
    auto listener = std::make_shared<CustomParticipantListener>(node_name, "/mas");
    RCLCPP_INFO(listener->get_logger(), "started");
    rclcpp::spin(listener);
    RCLCPP_INFO(listener->get_logger(), "shutdown...");
    listener->shutdown();
    rclcpp::shutdown();
    printf("bye!\n");
    return 0;
}
