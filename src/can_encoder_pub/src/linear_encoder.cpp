#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
int main(int argc, char **argv)
{

    ros::init(argc, argv, "can_encoder");
    ros::NodeHandle n;
    ros::Publisher can_encoder = n.advertise<geometry_msgs::Twist>("can_encoder", 10);

    ros::Rate loop_rate(10);

    int s, i;
    int nbytes, nbytes2, nbytes3, nbytes4, nbytes5, nbytes6, nbytes7, nbytes8, nbytes9, nbytes10/**, nbytes11, nbytes12, nbytes13, nbytes14, nbytes15, nbytes16,
        nbytes17, nbytes18, nbytes19, nbytes20, nbytes21, nbytes22, nbytes23, nbytes24, nbytes25, nbytes26, nbytes27, nbytes28, nbytes29, nbytes30, nbytes31, nbytes32**/;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_frame frame2;
    struct can_frame frame3;
    struct can_frame frame4;
    struct can_frame frame5;
    struct can_frame frame6;
    struct can_frame frame7;
    struct can_frame frame8;
    struct can_frame frame9;
    struct can_frame frame10;
    /**
    struct can_frame frame11;
    struct can_frame frame12;
    struct can_frame frame13;
    struct can_frame frame14;
    struct can_frame frame15;
    struct can_frame frame16;
    struct can_frame frame17;
    struct can_frame frame18;
    
    struct can_frame frame19;
    struct can_frame frame20;
    struct can_frame frame21;
    struct can_frame frame22;
    struct can_frame frame23;
    struct can_frame frame24;
    
    struct can_frame frame25;
    struct can_frame frame26;
    struct can_frame frame27;
    struct can_frame frame28;
    struct can_frame frame29;
    struct can_frame frame30;
    struct can_frame frame31;
    struct can_frame frame32;
    **/
    // struct can_frame frame1;

    printf("CAN Sockets Receive Demo\r\n");

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Socket");
        return 1;
    }

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return 1;
    }

    printf("before while loop\r\n");

    while (ros::ok())
    {
        //std_msgs::Float32 msg;
        geometry_msgs::Twist msg;
        struct can_frame can_array[10];

        printf("before read\r\n");

        nbytes = read(s, &frame, sizeof(struct can_frame));
        nbytes2 = read(s, &frame2, sizeof(struct can_frame));
        nbytes3 = read(s, &frame3, sizeof(struct can_frame));
        nbytes4 = read(s, &frame4, sizeof(struct can_frame));
        nbytes5 = read(s, &frame5, sizeof(struct can_frame));
        nbytes6 = read(s, &frame6, sizeof(struct can_frame));
        nbytes7 = read(s, &frame7, sizeof(struct can_frame));
        nbytes8 = read(s, &frame8, sizeof(struct can_frame));
        nbytes9 = read(s, &frame9, sizeof(struct can_frame));
        nbytes10 = read(s, &frame10, sizeof(struct can_frame));
        /**
        nbytes11 = read(s, &frame11, sizeof(struct can_frame));
        nbytes12 = read(s, &frame12, sizeof(struct can_frame));
        nbytes13 = read(s, &frame13, sizeof(struct can_frame));
        nbytes14 = read(s, &frame14, sizeof(struct can_frame));
        nbytes15 = read(s, &frame15, sizeof(struct can_frame));
        nbytes16 = read(s, &frame16, sizeof(struct can_frame));
        nbytes17 = read(s, &frame17, sizeof(struct can_frame));
        nbytes18 = read(s, &frame18, sizeof(struct can_frame));
        
        nbytes19 = read(s, &frame19, sizeof(struct can_frame));
        nbytes20 = read(s, &frame20, sizeof(struct can_frame));
        nbytes21 = read(s, &frame21, sizeof(struct can_frame));
        nbytes22 = read(s, &frame22, sizeof(struct can_frame));
        nbytes23 = read(s, &frame23, sizeof(struct can_frame));
        nbytes24 = read(s, &frame24, sizeof(struct can_frame));
        
        nbytes25 = read(s, &frame25, sizeof(struct can_frame));
        nbytes26 = read(s, &frame26, sizeof(struct can_frame));
        nbytes27 = read(s, &frame27, sizeof(struct can_frame));
        nbytes28 = read(s, &frame28, sizeof(struct can_frame));
        nbytes29 = read(s, &frame29, sizeof(struct can_frame));
        nbytes30 = read(s, &frame30, sizeof(struct can_frame));
        nbytes31 = read(s, &frame31, sizeof(struct can_frame));
        nbytes32 = read(s, &frame32, sizeof(struct can_frame));**/

        can_array[0] = frame;
        can_array[1] = frame2;
        can_array[2] = frame3;
        can_array[3] = frame4;
        can_array[4] = frame5;
        can_array[5] = frame6;
        can_array[6] = frame7;
        can_array[7] = frame8;
        can_array[8] = frame9;
        can_array[9] = frame10;

        printf("CAN array\r");
        /**
        can_array[10] = frame11;
        can_array[11] = frame12;
        can_array[12] = frame13;
        can_array[13] = frame14;
        can_array[14] = frame15;
        can_array[15] = frame16;
        can_array[16] = frame17;
        can_array[17] = frame18;
        
        can_array[18] = frame19;
        can_array[19] = frame20;
        can_array[20] = frame21;
        can_array[21] = frame22;
        can_array[22] = frame23;
        can_array[23] = frame24;
        
        can_array[24] = frame25;
        can_array[25] = frame26;
        can_array[26] = frame27;
        can_array[27] = frame28;
        can_array[28] = frame29;
        can_array[29] = frame30;
        can_array[30] = frame31;
        can_array[31] = frame32;**/

        printf("before if\r\n");

        if (nbytes < 0 or nbytes2 < 0 or nbytes3 < 0 or nbytes4 < 0 or nbytes5 < 0 or nbytes6 < 0 or nbytes7 < 0 or nbytes8 < 0 or nbytes9 < 0 or nbytes10 < 0/** or nbytes11 < 0 or nbytes12 < 0 or nbytes13 < 0 or nbytes14 < 0 or nbytes15 < 0 or nbytes16 < 0 or 
            nbytes < 17 or nbytes18 < 0/** or nbytes19 < 0 or nbytes20 < 0 or nbytes21 < 0 or nbytes22 < 0 or nbytes23 < 0 or nbytes24 < 0 or nbytes25 < 0 or nbytes26 < 0 or nbytes27 < 0 or nbytes28 < 0 or nbytes29 < 0 or nbytes30 < 0 or nbytes31 < 0 or nbytes32 < 0**/)
        {
            perror("Read");
            //return 1;
        }

        // printf("0x%03X [%d] ", frame.can_id, frame.can_dlc);
        // printf("0x%03X [%d] ", frame2.can_id, frame.can_dlc);
        // printf("0x%03X [%d] ", frame3.can_id, frame.can_dlc);
        // printf("0x%03X [%d] ", frame4.can_id, frame.can_dlc);

        //int data;
        // for (i = 0; i < frame.can_dlc; i++){
        //     printf("%02X ", frame.data[i]);
        // };
        //int pos = (int)(frame.can_dlc);
        //data =  (int)(frame.data[0]);

        for (i = 0; i < sizeof(can_array); i++)
        {
            //////////////////////////////////////////////// For Linear Encoders /////////////////////////////////////////////////////////
            if (can_array[i].can_id == 0x190)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint position = (b << 8) | (a);
                double positionFloat = position;
                positionFloat = (positionFloat / 40.96 + 325)/1000 + 0.032;
                std::cout << positionFloat << "\n";
                msg.angular.y = positionFloat;
            }
            else if (can_array[i].can_id == 0x191)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint position = (b << 8) | (a);
                double positionFloat = position;
                positionFloat = (positionFloat / 40.96 + 330)/1000 + 0.032;
                std::cout << positionFloat << "\n";
                msg.angular.z = positionFloat;
            }
            //***************************************************************************************************************************//

            /////////////////////////////////////////////////For Rotational Encoders//////////////////////////////////////////////////////

            else if (can_array[i].can_id == 0x181)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint c = can_array[i].data[2];
                uint d = can_array[i].data[3];
                uint position = (d << 24) | (c << 16) | (b << 8) | (a);
                double positionFloat = position;
                positionFloat = ((positionFloat - 67063944) * 0.02197) + 87.5; //67063944
                std::cout << positionFloat << "\n";
                msg.linear.x = positionFloat;
            }

            else if (can_array[i].can_id == 0x182)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint c = can_array[i].data[2];
                uint d = can_array[i].data[3];
                uint position = (d << 24) | (c << 16) | (b << 8) | (a);
                double positionFloat = position;
                positionFloat = ((positionFloat - 137458) * 0.02197) - 1482.5;
                std::cout << positionFloat << "\n";
                msg.linear.y = positionFloat;
            }

            else if (can_array[i].can_id == 0x183)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint c = can_array[i].data[2];
                uint d = can_array[i].data[3];
                uint position = (d << 24) | (c << 16) | (b << 8) | (a);
                double positionFloat = position;
                positionFloat = ((positionFloat - 241754) * 0.02197) - 32.0;
                std::cout << positionFloat << "\n";
                msg.linear.z = positionFloat;
            }

            else if (can_array[i].can_id == 0x185)
            {
                uint a = can_array[i].data[0];
                uint b = can_array[i].data[1];
                uint c = can_array[i].data[2];
                uint d = can_array[i].data[3];
                uint position = (d << 24) | (c << 16) | (b << 8) | (a);
                double positionFloat = position;
                positionFloat = ((positionFloat - 13977) * 0.02197) - 0.0;
                std::cout << positionFloat << "\n";
                msg.angular.x = positionFloat;
            }
        };

        can_encoder.publish(msg);

        //printf("\r\n");
        std::cout << msg << std::endl;
        //ROS_INFO("%s", msg.data.c_str())

        ros::spinOnce();
    }
    return 0;
}