<robot name="Xtellar.Robotics/ToGo.Prototype">

    <!-- mujoco tags for collision detection -->
@[if _mujoco]@
    <mujoco>
        <compiler meshdir="../meshes" />
        <contact>
            <exclude body1="L0_body_front"  body2="Lfl1_hip_roll" />
            <exclude body1="L0_body_front"  body2="Lfl2_hip_pitch" />
            <exclude body1="L0_body_front"  body2="Lfr1_hip_roll" />
            <exclude body1="L0_body_front"  body2="Lfr2_hip_pitch" />
            <exclude body1="L0_body_front"  body2="L1_body_rear" />
            <exclude body1="L1_body_rear"   body2="Lrl1_hip_roll" />
            <exclude body1="L1_body_rear"   body2="Lrl2_hip_pitch" />
            <exclude body1="L1_body_rear"   body2="Lrr1_hip_roll" />
            <exclude body1="L1_body_rear"   body2="Lrr2_hip_pitch" />
        </contact>
    </mujoco>
@[end if]@

    <material name="main">
        <color rgba="@_main_rgba[0] @_main_rgba[1] @_main_rgba[2] @_main_rgba[3]"/>
    </material>

    <!-- link tags  -->
    <link name='F_base' />

    <!-- body torso -->
    @(tag_link(link_name='L0_body_front',   mesh_path='l0_body_front.stl',  mesh_material='main', indent=1))
    @(tag_link(link_name='L1_body_rear',    mesh_path='l1_body_rear.stl',   mesh_material='main', indent=1))

    <!-- front left leg -->
    @(tag_link(link_name='Lfl1_hip_roll',   mesh_path='lfl1_hip_roll.stl',  mesh_material='main', indent=1))
    @(tag_link(link_name='Lfl2_hip_pitch',  mesh_path='lfl2_hip_pitch.stl', mesh_material='main', indent=1))
    @(tag_link(link_name='Lfl3_knee',       mesh_path='lfl3_knee.stl',      mesh_material='main', indent=1))
    @(tag_link(link_name='Lfl4_wheel',      mesh_path='lfl4_wheel.stl',     mesh_material='main', indent=1))

    <!-- front right leg -->
    @(tag_link(link_name='Lfr1_hip_roll',   mesh_path='lfr1_hip_roll.stl',  mesh_material='main', indent=1))
    @(tag_link(link_name='Lfr2_hip_pitch',  mesh_path='lfr2_hip_pitch.stl', mesh_material='main', indent=1))
    @(tag_link(link_name='Lfr3_knee',       mesh_path='lfr3_knee.stl',      mesh_material='main', indent=1))
    @(tag_link(link_name='Lfr4_wheel',      mesh_path='lfr4_wheel.stl',     mesh_material='main', indent=1))

    <!-- rear left leg -->
    @(tag_link(link_name='Lrl1_hip_roll',   mesh_path='lrl1_hip_roll.stl',  mesh_material='main', indent=1))
    @(tag_link(link_name='Lrl2_hip_pitch',  mesh_path='lrl2_hip_pitch.stl', mesh_material='main', indent=1))
    @(tag_link(link_name='Lrl3_knee',       mesh_path='lrl3_knee.stl',      mesh_material='main', indent=1))
    @(tag_link(link_name='Lrl4_wheel',      mesh_path='lrl4_wheel.stl',     mesh_material='main', indent=1))

    <!-- rear right leg -->
    @(tag_link(link_name='Lrr1_hip_roll',   mesh_path='lrr1_hip_roll.stl',  mesh_material='main', indent=1))
    @(tag_link(link_name='Lrr2_hip_pitch',  mesh_path='lrr2_hip_pitch.stl', mesh_material='main', indent=1))
    @(tag_link(link_name='Lrr3_knee',       mesh_path='lrr3_knee.stl',      mesh_material='main', indent=1))
    @(tag_link(link_name='Lrr4_wheel',      mesh_path='lrr4_wheel.stl',     mesh_material='main', indent=1))


    <!-- joint tags  -->

    <!-- body torso -->
    @(tag_joint(joint_name='J_F_base_L0_body_front', indent=1))
    @(tag_joint(joint_name='J1_waist', indent=1))

    <!-- front left leg -->
    @(tag_joint(joint_name='Jfl1_hip_roll', indent=1))
    @(tag_joint(joint_name='Jfl2_hip_pitch', indent=1))
    @(tag_joint(joint_name='Jfl3_knee', indent=1))
    @(tag_joint(joint_name='Jfl4_wheel', indent=1))

    <!-- front right leg -->
    @(tag_joint(joint_name='Jfr1_hip_roll', indent=1))
    @(tag_joint(joint_name='Jfr2_hip_pitch', indent=1))
    @(tag_joint(joint_name='Jfr3_knee', indent=1))
    @(tag_joint(joint_name='Jfr4_wheel', indent=1))

    <!-- rear left leg -->
    @(tag_joint(joint_name='Jrl1_hip_roll', indent=1))
    @(tag_joint(joint_name='Jrl2_hip_pitch', indent=1))
    @(tag_joint(joint_name='Jrl3_knee', indent=1))
    @(tag_joint(joint_name='Jrl4_wheel', indent=1))

    <!-- rear left leg -->
    @(tag_joint(joint_name='Jrr1_hip_roll', indent=1))
    @(tag_joint(joint_name='Jrr2_hip_pitch', indent=1))
    @(tag_joint(joint_name='Jrr3_knee', indent=1))
    @(tag_joint(joint_name='Jrr4_wheel', indent=1))

</robot>
