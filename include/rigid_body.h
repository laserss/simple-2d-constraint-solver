//ifndef与define定义了一个宏，用于防止头文件被重复包含
#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H
    #define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H

    //namespace定义了一个命名空间，用于防止命名冲突
    //scs是Simple 2D Constraint Solver的缩写
    namespace atg_scs {
        struct RigidBody {
            public:
                RigidBody();
                ~RigidBody();

                void localToWorld(double x, double y, double *w_x, double *w_y);
                void worldToLocal(double x, double y, double *l_x, double *l_y);

                double p_x;//刚体本地坐标系原点的世界坐标x
                double p_y;//刚体本地坐标系原点的世界坐标y

                double v_x;
                double v_y;

                double theta;
                double v_theta;

                double m;
                double I;

                int index;//刚体的索引,用于在刚体系统中索引刚体。-1表示刚体未添加到系统中。

                //初始化函数，将所有成员变量设置为0（除了index，在构造函数中设置为-1）。在构造函数中调用。
                void reset();

                //const表示该函数不会修改成员变量
                //计算刚体的能量，包括动能和转动动能
                double energy() const;
        };
    } /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_H */
