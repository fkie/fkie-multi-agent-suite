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

#include "rmw_fastrtps_shared_cpp/guid_utils.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"

#include "fastrtps/Domain.h"
#include "fastrtps/participant/ParticipantListener.h"
#include "fastrtps/subscriber/SubscriberListener.h"

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/thread_safety_annotations.hpp"
#include "rmw/impl/cpp/key_value.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "fkie_mas_msgs/msg/changed_state.hpp"
#include "fkie_mas_msgs/msg/gid.hpp"
#include "fkie_mas_msgs/msg/participant_entities_info.hpp"
#include "fkie_mas_msgs/srv/get_participants.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"

using namespace std::chrono_literals;
using namespace fkie_mas_msgs::msg;

using participant_map_t = std::map<eprosima::fastrtps::rtps::GUID_t, ParticipantEntitiesInfo>;

/**
 * Get a value from a environment variable
 */
std::string getEnvironmentVariable(std::string const &key)
{
    char *val = getenv(key.c_str());
    return val == NULL ? std::string("") : std::string(val);
}

void convert_gid_to_msg(const eprosima::fastrtps::rtps::GUID_t &gid, fkie_mas_msgs::msg::Gid &msg_gid)
{
    rmw_fastrtps_shared_cpp::copy_from_fastrtps_guid_to_byte_array(gid, const_cast<uint8_t *>(msg_gid.data.begin()));
}

void convert_msg_to_gid(const fkie_mas_msgs::msg::Gid &msg_gid, eprosima::fastrtps::rtps::GUID_t &gid)
{
    rmw_fastrtps_shared_cpp::copy_from_byte_array_to_fastrtps_guid(const_cast<uint8_t *>(msg_gid.data.begin()), &gid);
}

class CustomParticipantListener : public rclcpp::Node,
                                  public eprosima::fastrtps::ParticipantListener,
                                  public eprosima::fastrtps::SubscriberListener
{
public:
    CustomParticipantListener(const std::string &node_name, const std::string &namespace_) : Node(node_name, namespace_),
                                                                                             eprosima::fastrtps::ParticipantListener(),
                                                                                             eprosima::fastrtps::SubscriberListener()
    {
        rclcpp::QoS qos_settings(10);
        // qos_settings.reliable();
        // qos_settings.transient_local();
        // qos_settings.keep_last(1);
        on_shutdown = false;
        RCLCPP_INFO(get_logger(), "Node name: %s", node_name.c_str());
        publisher_ = this->create_publisher<fkie_mas_msgs::msg::ChangedState>("~/changed", qos_settings);
        RCLCPP_INFO(get_logger(), "  publisher: %s [fkie_mas_msgs/msg/ChangedState]", this->publisher_->get_topic_name());
        service_ = this->create_service<fkie_mas_msgs::srv::GetParticipants>("~/get_participants",
                                                                             std::bind(&CustomParticipantListener::get_participants,
                                                                                       this,
                                                                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        RCLCPP_INFO(get_logger(), "  service: %s [fkie_mas_msgs/msg/GetParticipants]", this->service_->get_service_name());
        daemon_topic_ = std::string("rt") + this->publisher_->get_topic_name();
        participant_ = nullptr;
        eprosima::fastrtps::ParticipantAttributes participant_attr;
        std::string full_name = this->get_node_base_interface()->get_fully_qualified_name();
        RCLCPP_INFO(get_logger(), "create eProsima participant: %s", full_name.c_str());
        participant_attr.rtps.setName(full_name.erase(0, 1).c_str());
        char *ros_domain_id = getenv("ROS_DOMAIN_ID");
        if (ros_domain_id != NULL)
        {
            RCLCPP_INFO(get_logger(), "listen to domain id: %s", ros_domain_id);
            participant_attr.domainId = atoi(ros_domain_id);
        }
        participant_ = eprosima::fastrtps::Domain::createParticipant(participant_attr, this);
    }

    void shutdown()
    {
        this->on_shutdown = true;
        eprosima::fastrtps::Domain::stopAll();
    }

    // SubscriberListener implementation
    void onSubscriptionMatched(
        eprosima::fastrtps::Subscriber * /*sub*/,
        eprosima::fastrtps::rtps::MatchingInfo &info) final
    {
        // std::lock_guard<std::mutex> lock(internalMutex_);
        if (eprosima::fastrtps::rtps::MATCHED_MATCHING == info.status)
        {
            // std::cout << "matched subscription " << info.remoteEndpointGuid << std::endl;
            // publishers_.insert(info.remoteEndpointGuid);
        }
        else if (eprosima::fastrtps::rtps::REMOVED_MATCHING == info.status)
        {
            // publishers_.erase(info.remoteEndpointGuid);
            // std::cout << "removed subscription " << info.remoteEndpointGuid << std::endl;
        }
    }

    void onSubscriberDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) override
    {
        (void)participant;

        // if (eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER)
        //     {
        //         isnew = true;
        //     }
        //     else if (info.status == eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER)
        //     {
        //     }
        //     process_discovery_info(info.info, isnew, fkie_mas_msgs::msg::TopicEntity::INFO_READER);
        // }
        std::string name = info.info.topicName().c_str();
        publish_notification(2, name);
        // if (name.rfind("rq/", 0) != 0 && name.rfind("rr/", 0) != 0)
        // {
        // }
    }

    void onPublisherDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) override
    {
        (void)participant;

        // if (eprosima::fastrtps::rtps::WriterDiscoveryInfo::CHANGED_QOS_WRITER != info.status)
        // {
        //     bool isnew = false;
        //     if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::DISCOVERED_WRITER)
        //     {
        //         isnew = true;
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher discovered");
        //     }
        //     else if (info.status == eprosima::fastrtps::rtps::WriterDiscoveryInfo::REMOVED_WRITER)
        //     {
        //         RCLCPP_INFO(get_logger(), "rosgraph: publisher removed");
        //     }
        //     process_discovery_info(info.info, isnew, fkie_mas_msgs::msg::TopicEntity::INFO_WRITER);
        // }
        std::string name = info.info.topicName().c_str();
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

    void get_participants(const std::shared_ptr<rmw_request_id_t>,
                           const std::shared_ptr<fkie_mas_msgs::srv::GetParticipants::Request>,
                           const std::shared_ptr<fkie_mas_msgs::srv::GetParticipants::Response> response)
    {
        RCLCPP_INFO(get_logger(), "Request get_participants");
        std::lock_guard<std::mutex> guard(mutex_);
        for (auto it_dp = discoveredParticipants_.begin(); it_dp != discoveredParticipants_.end(); it_dp++)
        {
            response->participants.push_back(it_dp->second);
        }
        RCLCPP_INFO(get_logger(), " request get_participants handled with %lu participants", response->participants.size());
        RCLCPP_DEBUG(get_logger(), "rosgraph: service response state with %lu participants", response->participants.size());
    }

    void onParticipantDiscovery(
        eprosima::fastrtps::Participant * /*participant*/,
        eprosima::fastrtps::rtps::ParticipantDiscoveryInfo &&info)
    {
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery: participant info %s", info.info.m_participantName.c_str());
        RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:   status %d", info.status);
        auto participant_guid = to_string(info.info.m_guid);
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DISCOVERED_PARTICIPANT:
        {
            RCLCPP_INFO(get_logger(), "onParticipantDiscovery: new participant %s", participant_guid.c_str());
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            if (itp == discoveredParticipants_.end())
            {
                ParticipantEntitiesInfo pei;
                fkie_mas_msgs::msg::Gid gid;
                convert_gid_to_msg(info.info.m_guid, gid);
                pei.guid = gid;
                discoveredParticipants_[info.info.m_guid] = pei;
            }
            auto pi = discoveredParticipants_[info.info.m_guid];
            auto map = rmw::impl::cpp::parse_key_value(info.info.m_userData);
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
            for (auto i = info.info.default_locators.unicast.begin(); i != info.info.default_locators.unicast.end(); ++i)
            {
                RCLCPP_DEBUG(get_logger(), "onParticipantDiscovery:  add unicast locator: %s", to_string(*i).c_str());
                pi.unicast_locators.push_back(to_string(*i));
            }
            discoveredParticipants_[info.info.m_guid] = pi;
            break;
        }
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::REMOVED_PARTICIPANT:
        // fall through
        case eprosima::fastrtps::rtps::ParticipantDiscoveryInfo::DROPPED_PARTICIPANT:
        {
            std::lock_guard<std::mutex> guard(mutex_);
            auto itp = discoveredParticipants_.find(info.info.m_guid);
            // only consider known GUIDs
            if (itp != discoveredParticipants_.end())
            {
                RCLCPP_INFO(get_logger(), "onParticipantDiscovery: remove participant %s:", to_string(info.info.m_guid).c_str());
                discoveredParticipants_.erase(itp);
            }
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
    eprosima::fastrtps::Participant *participant_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
    rclcpp::Publisher<fkie_mas_msgs::msg::ChangedState>::SharedPtr publisher_;
    rclcpp::Service<fkie_mas_msgs::srv::GetParticipants>::SharedPtr service_;
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
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", true);
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
    // std::string ros_distro = getEnvironmentVariable("ROS_DISTRO");

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
