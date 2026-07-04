# Simple 2D Constraint Solver

This is a simple constraint solver and physics engine written in C++.

To see it in action, check out the [demo](https://github.com/ange-yaghi/scs-2d-demo).


--------------------------------
save point:
device:     mac
time:       2026/5/28

location:   gaussian_elimination_sle_solver.h/cpp  
target:     gaussian_elimination_sle_solver.h/cpp
            gauss_seidel_sle_solver.h/cpp
            conjugate_gradient_sle_solver.h/cpp

done:       rigid_body.h/cpp
            force_generator.h/cpp
            gravity_force_generator.h/cpp   260414
            utilities.h/cpp                 260414
            system_state.h/cpp              260415
            spring.h/cpp                    260415
            constraint.h/cpp                260415
            link_constraint.h/cpp           260508
            matrix.h/cpp                    260522
            sparse_matrix.h/cpp             260526
            sle_solver.h/cpp                260527
            generic_rigid_body_system.h/cpp 260704





--------------------------------
how to save:
1、update "save point" in readme.md
2、git add . (改动添加至暂存区)
3、git commit -m "提交说明"  (将暂存区内容保存至本地仓库)
4、git push origin master


how to load:

turn on GitBash(Windows)/Terminal(Mac)

GitBash:
cd /d/dev/simple-2d-constraint-solver

Terminal:
cd /Users/laserss/Desktop/simple-2d-constraint-solver


IF switch device
    git pull origin master

