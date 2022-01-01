#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <errno.h>
#include <pthread.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace std;

// buffers
char recv_buffer[1024];

// locks
pthread_mutex_t m_r_buffer;

// semaphores
pthread_cond_t s_recv;

void* server_socket(void* arg) {
    //create socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    //bind socket
    struct sockaddr_in myServer;
    myServer.sin_family = AF_INET;
    myServer.sin_port = htons(5402);
    myServer.sin_addr.s_addr = INADDR_ANY;
    int bnd = bind(fd, (struct sockaddr*)&myServer, sizeof(myServer));

    //listen on
    bnd = listen(fd, 128);

    ROS_INFO("Server publisher ready\n");

    struct sockaddr_in client;
    socklen_t addrlen = sizeof(sockaddr_in);
    while (true) {
        int cnt = accept(fd, (struct sockaddr*)&client, &addrlen);
        char ip[32];
        ROS_INFO("Client: %s connected.\t port: %d \n", inet_ntop(AF_INET, &client.sin_addr.s_addr,  ip, sizeof(ip)), ntohs(client.sin_port));

        // communicate
        while (true) {
            ROS_INFO("Buffer: %s\n", recv_buffer);
            //receive message
            pthread_mutex_lock(&m_r_buffer);
            if (strlen(recv_buffer) != 0) pthread_cond_wait(&s_recv, &m_r_buffer); // if still not publish, wait for publisher
            int len = recv(cnt, recv_buffer, sizeof(recv_buffer), 0);
            ROS_INFO("Received length: %d\n", len);
            if (len > 0) {
                ROS_INFO("Client sent: %s\n", recv_buffer);
            }
            else if (len == 0) {
                ROS_INFO("Client disconnected.\n");
                close(cnt);
                close(fd);
                break;
            }
            else {
                close(cnt);
                close(fd);
                break;
            }
            pthread_cond_signal(&s_recv); // call publisher
            pthread_mutex_unlock(&m_r_buffer);
        }
    }
    pthread_mutex_unlock(&m_r_buffer);
    close(fd);
}

int main(int argc, char** argv) {
    //initialize ros
    ros::init(argc, argv, "uav_server_publisher");
    ros::NodeHandle n;

    //initialize lock
    pthread_mutex_init(&m_r_buffer, NULL);
    
    //initialize semaphore
    pthread_cond_init(&s_recv, NULL);

    //initialize buffer
    memset(recv_buffer, '\0', sizeof(recv_buffer));

    // ros publisher
    ros::Publisher pub = n.advertise<std_msgs::String>("uav_message_in", 1000);
    ros::Rate loop_rate(10);

    // start socket
    pthread_t socket;
    pthread_create(&socket, NULL, server_socket, NULL);

    // 循环向topic发布数据
    while(ros::ok) {
        std_msgs::String msg;
        std::stringstream ss;

        pthread_mutex_lock(&m_r_buffer);
        if (strcmp(recv_buffer, "") == 0) pthread_cond_wait(&s_recv, &m_r_buffer); // if empty, wait for socket
        ss << recv_buffer;
        msg.data = ss.str();
        ROS_INFO("publish %s", msg.data.c_str());
        pub.publish(msg);
        memset(recv_buffer, '\0', sizeof(recv_buffer)); // empty the buffer
        pthread_cond_signal(&s_recv); // call socket to fill the buffer
        pthread_mutex_unlock(&m_r_buffer);
    }

    // release resource
    pthread_mutex_destroy(&m_r_buffer);
    pthread_cond_destroy(&s_recv);

    return 0;
}
