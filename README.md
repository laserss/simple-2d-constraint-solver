# Simple 2D Constraint Solver

This is a simple constraint solver and physics engine written in C++.

To see it in action, check out the [demo](https://github.com/ange-yaghi/scs-2d-demo).


--------------------------------
save point:
device:     mac
time:       2026/5/28

location:   generic_rigid_body_system.cpp  line:196
target:     generic_rigid_body_system.cpp 第二步：组装方程组右端项 169-177 
            第三步：求解线性方程组，得到 λ 181-188 
            第四步：计算约束力 198-206 
            第五步：叠加到加速度，再除以质量 208-235.
            最终每个刚体的加速度 = （外力 + 约束力）/ 质量，交给 ODE 求解器积分推进时间步。

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

