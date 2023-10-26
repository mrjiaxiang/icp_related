#include "iterative_closest_point.h"

using namespace std;

#ifndef FLT_MAX
    #define FLT_MAX 3.402823466e+38F   // max value
#endif

IterativeClosestPoint::IterativeClosestPoint() {
    offset_other_map_points_ = 0;
    threshold_dist_          = 0.3f;
    threshold_angle_         = (float)DEG2RAD(3);

    op_decimate_other_map_points_ = 1;
    op_ALFA_                      = 0.8f;
    op_min_abs_trans_step_        = 1e-6f;
    op_min_abs_rot_step_          = 1e-6f;
    op_smallest_threshold_dist_   = 0.005f;
    op_max_iterations_            = 150;

    points_x_     = NULL;
    points_y_     = NULL;
    new_points_x_ = NULL;
    new_points_y_ = NULL;
    match_pairs_  = NULL;
    points_count_ = 0;

    icp_result_pose_ = Pose2D(0, 0, 0);

    target_x_int_ = NULL;
    target_y_int_ = NULL;
    target_max_y_ = target_max_x_ = (std::numeric_limits<float>::min)();   // FLT_MIN
    target_min_y_ = target_min_x_ = (std::numeric_limits<float>::max)();   // FLT_MAX
    tree_                         = new rbox::RTree2i;
}

IterativeClosestPoint::~IterativeClosestPoint() {
    if (points_x_) {
        delete[] points_x_;
        points_x_ = NULL;
    }

    if (points_y_) {
        delete[] points_y_;
        points_y_ = NULL;
    }

    if (match_pairs_) {
        delete[] match_pairs_;
        match_pairs_ = NULL;
    }

    if (new_points_x_) {
        delete[] new_points_x_;
        new_points_x_ = NULL;
    }

    if (new_points_y_) {
        delete[] new_points_y_;
        new_points_y_ = NULL;
    }

    if (target_x_int_) {
        delete[] target_x_int_;
        target_x_int_ = NULL;
    }

    if (target_y_int_) {
        delete[] target_y_int_;
        target_y_int_ = NULL;
    }

    if (tree_) {
        delete tree_;
        tree_ = NULL;
    }
}

int IterativeClosestPoint::RTreeNearest(int *pos_in, int *pos_out, int32_t *index) {
    vector<int> search_out;
    int i;
    tree_->KNN(pos_in, 1, search_out);
    if (search_out.size() > 0) {
        i = search_out[0];
    } else {
        i = 0;
        printf("RTreeNearest not fount NearPoint.\n");
    }

    if (pos_out) {
        pos_out[0] = target_x_int_[i];
        pos_out[1] = target_y_int_[i];
    }

    *index = i;

    return ((pos_in[0] - target_x_int_[i]) * (pos_in[0] - target_x_int_[i]) +
            (pos_in[1] - target_y_int_[i]) * (pos_in[1] - target_y_int_[i]));
}

bool IterativeClosestPoint::SetInputSource(float *source_x_arr, float *source_y_arr, int32_t source_point_count) {
    if (points_x_) {
        delete[] points_x_;
        points_x_ = NULL;
    }

    if (points_y_) {
        delete[] points_y_;
        points_y_ = NULL;
    }

    if (match_pairs_) {
        delete[] match_pairs_;
        match_pairs_ = NULL;
    }

    if (new_points_x_) {
        delete[] new_points_x_;
        new_points_x_ = NULL;
    }

    if (new_points_y_) {
        delete[] new_points_y_;
        new_points_y_ = NULL;
    }

    new_points_x_ = new float[source_point_count];
    new_points_y_ = new float[source_point_count];
    points_x_     = new float[source_point_count];
    points_y_     = new float[source_point_count];

    match_pairs_  = new MatchPair[source_point_count];
    points_count_ = source_point_count;

    memcpy(points_x_, source_x_arr, source_point_count * sizeof(float));
    memcpy(points_y_, source_y_arr, source_point_count * sizeof(float));
    return true;
}

bool IterativeClosestPoint::SetInputTarget(float *target_x_arr, float *target_y_arr, int32_t target_point_count) {
    if (target_x_int_) {
        delete[] target_x_int_;
        target_x_int_ = NULL;
    }
    if (target_y_int_) {
        delete[] target_y_int_;
        target_y_int_ = NULL;
    }

    target_x_int_ = new int[target_point_count];
    target_y_int_ = new int[target_point_count];

    for (int i = 0; i < target_point_count; i++) {
        float xy[2];
        int xy1000[2];
        xy[0]            = target_x_arr[i];
        xy[1]            = target_y_arr[i];
        xy1000[0]        = xy[0] * 1000;
        xy1000[1]        = xy[1] * 1000;
        target_x_int_[i] = xy1000[0];
        target_y_int_[i] = xy1000[1];

        if (xy[0] > target_max_x_) {
            target_max_x_ = xy[0];
        }
        if (xy[0] < target_min_x_) {
            target_min_x_ = xy[0];
        }
        if (xy[1] > target_max_y_) {
            target_max_y_ = xy[1];
        }
        if (xy[1] < target_min_y_) {
            target_min_y_ = xy[1];
        }

        tree_->Insert(xy1000, xy1000, i);
    }

    if (target_point_count > 0) {
        return true;
    } else {
        return false;
    }
}

Pose2D IterativeClosestPoint::GetFinalTransformation() { return icp_result_pose_; }

bool IterativeClosestPoint::Align(Pose2D estimate_pose_in) {
    int32_t match_count   = 0;
    bool keep_approaching = false;

    Pose2D mean                          = estimate_pose_in;
    Pose2D last_mean                     = mean;
    int32_t iter_count                   = 0;
    offset_other_map_points_             = 0;
    max_angular_dist_for_correspondence_ = threshold_angle_;
    max_dist_for_correspondence_         = threshold_dist_;

    if (op_decimate_other_map_points_ > 1) {
        op_decimate_other_map_points_ = 1;
    }
    if (op_decimate_other_map_points_ < 2) {
        op_decimate_other_map_points_ = 2;
    }

    float avg_dis_sq = .0f;

    do {
        keep_approaching      = false;
        angular_dist_pivot_x_ = (float)mean.x;
        angular_dist_pivot_y_ = (float)mean.y;
        match_count = DetermineMatching(points_x_, points_y_, points_count_, &mean, match_pairs_, &avg_dis_sq);

        if (match_count) {
            LeastSquareErrorRigidTransformation(match_pairs_, match_count, &mean);

            keep_approaching = true;

            if (!(fabs(last_mean.x - mean.x) > op_min_abs_trans_step_ ||
                  fabs(last_mean.y - mean.y) > op_min_abs_trans_step_ ||
                  fabs(mean.WrapToPi(last_mean.Phi() - mean.Phi())) > op_min_abs_rot_step_)) {
                max_angular_dist_for_correspondence_ *= op_ALFA_;
                max_dist_for_correspondence_ *= op_ALFA_;

                if (max_dist_for_correspondence_ < op_smallest_threshold_dist_)
                    keep_approaching = false;

                if (++offset_other_map_points_ >= op_decimate_other_map_points_)
                    offset_other_map_points_ = 0;
            }

            last_mean = mean;
            iter_count++;

            if (iter_count >= op_max_iterations_ && max_dist_for_correspondence_ > op_smallest_threshold_dist_)
                max_dist_for_correspondence_ *= op_ALFA_;
        }
    } while ((keep_approaching && iter_count < op_max_iterations_) ||
             (iter_count >= op_max_iterations_ && max_dist_for_correspondence_ > op_smallest_threshold_dist_));

    icp_result_pose_ = mean;

    uint32_t current_op_decimate_other_map_points = op_decimate_other_map_points_;
    op_decimate_other_map_points_                 = 2;
    match_count = DetermineMatching(points_x_, points_y_, points_count_, &mean, match_pairs_, &avg_dis_sq);

    float TotalGoodness = 0;
    int last_id         = match_pairs_->other_id;
    float str_val       = 1;

    for (int i = 1; i < match_count; i++) {
        MatchPair *p = match_pairs_ + i;
        if (p->other_id - last_id <= 4) {
            if (str_val < 3325) {
                str_val *= 1.5;
            } else {
                TotalGoodness += str_val;
                str_val = 1;
            }
        } else {
            TotalGoodness += str_val;
            str_val = 1;
        }
        last_id = p->other_id;
    }
    if (str_val != 1) {
        TotalGoodness += str_val;
    }

    int32_t goodness       = 0;
    int32_t total_goodness = 0;
    if (points_count_ == 0) {
        goodness = 0;
    } else {
        goodness = (op_decimate_other_map_points_ * match_count) * 100 / points_count_;
    }

    op_decimate_other_map_points_ = current_op_decimate_other_map_points;
    total_goodness                = (int32_t)TotalGoodness;

    return true;
}

int32_t IterativeClosestPoint::DetermineMatching(float *other_x, float *other_y, int32_t points_count,
                                                 Pose2D *other_pose, MatchPair *pairs, float *avg_dist_sq) {
    int32_t match_count = 0;
    int sum_dist_sq     = 0;
    float obs_bound[4]  = {-FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX};

    if (!points_count) {
        return 0;
    }

    Pose2D local_pose;

    float *x_locals = new_points_x_;
    float *y_locals = new_points_y_;

    float sin_phi = sin(other_pose->Phi());
    float cos_phi = cos(other_pose->Phi());

    for (int32_t i = 0; i < points_count; i++) {
        float x_value = other_pose->x + cos_phi * other_x[i] - sin_phi * other_y[i];
        if (obs_bound[1] > x_value) {
            obs_bound[1] = x_value;
        }
        if (obs_bound[0] < x_value) {
            obs_bound[0] = x_value;
        }
        x_locals[i] = x_value;

        float y_value = other_pose->y + sin_phi * other_x[i] + cos_phi * other_y[i];
        if (obs_bound[3] > y_value) {
            obs_bound[3] = y_value;
        }
        if (obs_bound[2] < y_value) {
            obs_bound[2] = y_value;
        }
        y_locals[i] = y_value;
    }

    if (obs_bound[1] > target_max_x_ || obs_bound[0] < target_min_x_ || obs_bound[3] > target_max_y_ ||
        obs_bound[2] < target_min_y_) {
        return 0;
    }

    int mAngularDistPivotX_INT        = angular_dist_pivot_x_ * 1000;
    int mAngularDistPivotY_INT        = angular_dist_pivot_y_ * 1000;
    int mMaxDistForCorrespondence_INT = max_dist_for_correspondence_ * 1000;

    int itTime = 0;

    for (int32_t i = offset_other_map_points_; i < points_count; i += op_decimate_other_map_points_) {
        // For speed-up:
        int x_local = x_locals[i] * 1000;
        int y_local = y_locals[i] * 1000;

        int32_t this_i;
        int pos_in[2]     = {x_local, y_local};
        int match_dist_sq = RTreeNearest(pos_in, NULL, &this_i);

        int dist = (int)(max_angular_dist_for_correspondence_ *
                         sqrt((mAngularDistPivotX_INT - x_local) * (mAngularDistPivotX_INT - x_local) +
                              (mAngularDistPivotY_INT - y_local) * (mAngularDistPivotY_INT - y_local))) +
                   mMaxDistForCorrespondence_INT;

        int dist_sq = dist * dist;

        if (match_dist_sq < dist_sq) {
            // Save all the correspondences:
            MatchPair *p = pairs + match_count;

            p->this_id  = this_i;
            p->other_id = i;
            p->dist_sq  = match_dist_sq;

            match_count++;
            sum_dist_sq += match_dist_sq;
        }
        itTime++;
    }

    sum_dist_sq = match_count ? sum_dist_sq / match_count : 0;
    if (avg_dist_sq != NULL)
        *avg_dist_sq = sum_dist_sq;

    return match_count;
}

bool IterativeClosestPoint::LeastSquareErrorRigidTransformation(MatchPair *pairs, int32_t match_count, Pose2D *pose) {
    const size_t N = match_count;
    if (N < 2)
        return false;
    const float N_inv = 1.0f / N;   // For efficiency, keep this value.

    int SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
    float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;

    const int *this_px = target_x_int_;
    const int *this_py = target_y_int_;
    for (int32_t i = 0; i < match_count; i++) {
        MatchPair *corrIt = pairs + i;
        // Get the pair of points in the correspondence:
        int32_t this_i  = corrIt->this_id;
        int32_t other_i = corrIt->other_id;
        const int xa    = this_px[this_i];
        const int ya    = this_py[this_i];
        const int xb    = points_x_[other_i] * 1000;
        const int yb    = points_y_[other_i] * 1000;

        // Compute the terms:
        SumXa += xa;
        SumYa += ya;

        SumXb += xb;
        SumYb += yb;

        Sxx += xa * xb;
        Sxy += xa * yb;
        Syx += ya * xb;
        Syy += ya * yb;

    }   // End of "for all correspondences"...

    const float SumXa_f = SumXa;
    const float SumYa_f = SumYa;
    const float SumXb_f = SumXb;
    const float SumYb_f = SumYb;

    const float mean_x_a = SumXa_f * N_inv;
    const float mean_y_a = SumYa_f * N_inv;
    const float mean_x_b = SumXb_f * N_inv;
    const float mean_y_b = SumYb_f * N_inv;
    const float N_f      = N;

    // Auxiliary variables Ax,Ay:
    const float Ax = N_f * (Sxx + Syy) - SumXa_f * SumXb_f - SumYa_f * SumYb_f;
    const float Ay = SumXa_f * SumYb_f + N_f * (Syx - Sxy) - SumXb_f * SumYa_f;

    // printf("%d   %d   %d   %d   %f   %f   %f   %f\n", SumXa, SumXb, SumYa, SumYb, N*(Sxx + Syy), Ax, Ay, (Ax != 0 ||
    // Ay != 0) ? atan2(Ay, Ax) : 0);

    pose->SetPhi((Ax != 0 || Ay != 0) ? atan2(Ay, Ax) : 0);

    float sin_phi = sin(pose->Phi());
    float cos_phi = cos(pose->Phi());

    pose->x = (mean_x_a - mean_x_b * cos_phi + mean_y_b * sin_phi) / 1000;
    pose->y = (mean_y_a - mean_x_b * sin_phi - mean_y_b * cos_phi) / 1000;

    return true;
}
