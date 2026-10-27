// Offline regression: link against the rebuilt SDK; no controller connection.
#include "FXAxisPln.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
    CAxisJointPln planner;
    Vect8 neg, pos, vel, acc;
    for (int j = 0; j < 8; ++j) {
        neg[j] = -180; pos[j] = 180; vel[j] = 100; acc[j] = 100;
    }
    assert(planner.OnSetLmt(7, neg, pos, vel, acc));
    planner.OnSetFreq(50);
    // Reuse the same planner across moves, including a repeated goal.
    for (double delta : {1.0, 0.0, 0.01, -0.05, 0.099, 0.1}) {
        Vect7 start = {0}, target = {0};
        target[0] = delta;
        CPointSet points;
        assert(planner.OnMovJoint(0, start, target, .05, .02, &points));
        long count = points.OnGetPointNum();
        assert(count > 0);
        for (long i = 0; i < count; ++i) {
            double* q = points.OnGetPoint(i);
            for (int j = 0; j < 7; ++j) {
                assert(std::isfinite(q[j]));
                assert(q[j] >= std::fmin(start[j], target[j]) - 1e-10);
                assert(q[j] <= std::fmax(start[j], target[j]) + 1e-10);
            }
        }
        assert(std::fabs(points.OnGetPoint(count - 1)[0] - delta) < 1e-10);
    }
    Vect7 start = {0}, target = {181};
    CPointSet points;
    assert(!planner.OnMovJoint(0, start, target, .05, .02, &points));
    start[0] = 181; target[0] = 0;
    assert(!planner.OnMovJoint(0, start, target, .05, .02, &points));
    Vect8 from = {0}, to = {1};
    int slow = planner.OnPln(from, to, .05, .01);
    int fast = planner.OnPln(from, to, .05, .1);
    assert(slow > fast && fast > 0);
    std::cout << "joint planner regression passed\n";
}
