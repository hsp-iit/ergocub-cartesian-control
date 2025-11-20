struct YARPMatrix{}
(
    yarp.name = "yarp::sig::Matrix"
    yarp.includefile="yarp/sig/Matrix.h"
)

service ergoCubBimanualService
{
    bool go_to_pose(1: double x, 2: double y, 3: double z, 4: double q_x, 5: double q_y, 6: double q_z, 7: double q_w, 8: string arm);

    bool flat_go_to_pose(1: double x, 2: double y, 3: double z, 4: double m1, 5: double m2, 6: double m3, 7: double m4, 8: double m5, 9: double m6, 10: double m7, 11: double m8, 12: double m9, 13: string arm);

    bool go_to_position(1: double x, 2: double y, 3: double z, 4: string arm);

    bool rotate_rad(1: double angle, 2: double x, 3: double y, 4: double z, 5: string arm);

    bool rotate_deg(1: double angle, 2: double x, 3: double y, 4: double z, 5: string arm);

    YARPMatrix get_pose(1: string arm);

    bool go_home();

    bool stop();
}