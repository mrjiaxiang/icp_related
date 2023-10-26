#ifndef ITERATIVE_CLOSESET_POINT_H_
#define ITERATIVE_CLOSESET_POINT_H_

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <limits>
#include <vector>

#include "pose2d.h"
#include "rtree.h"

struct MatchPair {
    int32_t this_id;
    int32_t other_id;
    int dist_sq;
};

class IterativeClosestPoint {
 public:
    /**
     * @brief Construct a new Iterative Closest Point object
     */
    IterativeClosestPoint();

    /**
     * @brief Destroy the Iterative Closest Point object
     */
    ~IterativeClosestPoint();

    /**
     * @brief 设置源点云
     * @param source_x_arr
     * @param source_y_arr
     * @param source_point_count
     * @return true
     * @return false
     */
    bool SetInputSource(float *source_x_arr, float *source_y_arr, int32_t source_point_count);

    /**
     * @brief 设置目标点云
     * @param target_x_arr
     * @param target_y_arr
     * @param target_point_count
     * @return true
     * @return false
     */
    bool SetInputTarget(float *target_x_arr, float *target_y_arr, int32_t target_point_count);

    /**
     * @brief icp算法
     * @param estimate_pose_in
     * @return true
     * @return false
     */
    bool Align(Pose2D estimate_pose_in);

    /**
     * @brief 获取icp匹配后的坐标变换关系
     * @return Pose2D
     */
    Pose2D GetFinalTransformation();

 private:
    float *points_x_, *points_y_;
    float *new_points_x_, *new_points_y_;
    int *target_x_int_, *target_y_int_;
    MatchPair *match_pairs_;
    int32_t points_count_;
    Pose2D icp_result_pose_;

    rbox::RTree2i *tree_;
    float target_max_y_, target_max_x_, target_min_y_, target_min_x_;
    // Options
    uint32_t op_decimate_other_map_points_;
    float op_ALFA_;
    float op_min_abs_trans_step_, op_min_abs_rot_step_;
    float op_smallest_threshold_dist_;
    int32_t op_max_iterations_;

    // Params
    uint32_t offset_other_map_points_;
    float threshold_angle_, threshold_dist_;
    float max_angular_dist_for_correspondence_, max_dist_for_correspondence_;
    float angular_dist_pivot_x_, angular_dist_pivot_y_;

    int RTreeNearest(int *pos_in, int *pos_out, int32_t *index);

    int32_t DetermineMatching(float *other_x, float *other_y, int32_t points_count, Pose2D *other_pose,
                              MatchPair *pairs, float *avg_dist_sq = NULL);

    bool LeastSquareErrorRigidTransformation(MatchPair *pairs, int32_t match_count, Pose2D *pose);
};
#endif