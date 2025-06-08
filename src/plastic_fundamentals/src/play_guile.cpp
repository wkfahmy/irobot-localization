#include <ros/ros.h>
#include <create_fundamentals/StoreSong.h>
#include <create_fundamentals/PlaySong.h>
#include <unistd.h> // for sleep

int main(int argc, char **argv)
{
    ros::init(argc, argv, "guile_theme_player");
    ros::NodeHandle nh;

    ros::service::waitForService("store_song");
    ros::service::waitForService("play_song");

    ros::ServiceClient store_song = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
    ros::ServiceClient play_song = nh.serviceClient<create_fundamentals::PlaySong>("play_song");

    int MEASURE = 160;
    int Q = MEASURE/4;
    int HALF = MEASURE/2;

    std::vector<unsigned int> guile_song = {
        69, 9,
        74, 9,
        77, 9,
        38, 70,
        57, 33,
        36, 33,
        69, 9,
        74, 9,
        77, 9,
        67, 9,
        72, 9,
        76, 9,
        36, 33,
        67, 9,
        72, 9,
        76, 9,
        69, 69,
        74, 69,
        77, 73,
        43, 9,
        45, 9,
        43, 9,
        41, 9,
        38, 33,
        69, 9,
        74, 9,
        40, 9,
        67, 9,
        72, 9,
        76, 9,
        38, 9,
        36, 9,
    };

    create_fundamentals::StoreSong store_srv;
    store_srv.request.number = 1;
    store_srv.request.song = guile_song;
    if (store_song.call(store_srv)) {
        ROS_INFO("Guile theme uploaded!");
    } else {
        ROS_ERROR("Failed to upload song.");
        return 1;
    }

    sleep(1);

    create_fundamentals::PlaySong play_srv;
    play_srv.request.number = 1;
    if (play_song.call(play_srv)) {
        ROS_INFO("Guile theme playing!");
    } else {
        ROS_ERROR("Failed to play song.");
        return 1;
    }

    usleep((MEASURE/64.0 * 20) * 1e6);

    return 0;
}
