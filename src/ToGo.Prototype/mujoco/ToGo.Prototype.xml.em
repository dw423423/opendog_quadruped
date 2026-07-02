<mujoco model="Xtellar.Robotics/ToGo.Prototype">
    <compiler meshdir="../meshes" />

    <default>
        <default class="viscol">
            <geom
                type="mesh" rgba="@_main_rgba[0] @_main_rgba[1] @_main_rgba[2] @_main_rgba[3]"
                contype="1" conaffinity="1" density="0"
                solref="0.005 1" solimp="0.95 0.99 0.001 0.5 2"
            />
        </default>
        <default class="rubber_wheel">
            <geom
                type="mesh" rgba="@_main_rgba[0] @_main_rgba[1] @_main_rgba[2] @_main_rgba[3]"
                contype="1" conaffinity="1" density="0"
                solref="0.005 1" solimp="0.95 0.99 0.001 0.5 2"
                condim="6" friction="1.5 0.02 0.001"
            />
        </default>
    </default>

    <asset>
        <!-- body meshes -->
        <mesh name="mesh_body_front"     file="l0_body_front.stl"  scale="0.001 0.001 0.001" />
        <mesh name="mesh_body_rear"      file="l1_body_rear.stl"   scale="0.001 0.001 0.001" />

        <!-- front left leg meshes -->
        <mesh name="mesh_lfl1_hip_roll"  file="lfl1_hip_roll.stl"  scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfl2_hip_pitch" file="lfl2_hip_pitch.stl" scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfl3_knee"      file="lfl3_knee.stl"      scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfl4_wheel"     file="lfl4_wheel.stl"     scale="0.001 0.001 0.001" />

        <!-- front right leg meshes -->
        <mesh name="mesh_lfr1_hip_roll"  file="lfr1_hip_roll.stl"  scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfr2_hip_pitch" file="lfr2_hip_pitch.stl" scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfr3_knee"      file="lfr3_knee.stl"      scale="0.001 0.001 0.001" />
        <mesh name="mesh_lfr4_wheel"     file="lfr4_wheel.stl"     scale="0.001 0.001 0.001" />

        <!-- rear left leg meshes -->
        <mesh name="mesh_lrl1_hip_roll"  file="lrl1_hip_roll.stl"  scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrl2_hip_pitch" file="lrl2_hip_pitch.stl" scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrl3_knee"      file="lrl3_knee.stl"      scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrl4_wheel"     file="lrl4_wheel.stl"     scale="0.001 0.001 0.001" />

        <!-- rear right leg meshes -->
        <mesh name="mesh_lrr1_hip_roll"  file="lrr1_hip_roll.stl"  scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrr2_hip_pitch" file="lrr2_hip_pitch.stl" scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrr3_knee"      file="lrr3_knee.stl"      scale="0.001 0.001 0.001" />
        <mesh name="mesh_lrr4_wheel"     file="lrr4_wheel.stl"     scale="0.001 0.001 0.001" />
    </asset>

    <worldbody>
        <!-- robot floating base (root) -->
        @(taghead_body(link_name="F_base", indent=2))

            <!-- front body -->
            @(taghead_body(link_name="L0_body_front"))
                @(tag_inertial(link_name="L0_body_front", indent=4))
                <geom class="viscol" mesh="mesh_body_front" />

                <site name="imu" pos="0 0.13 0.05" euler="-90 0 -90" />

                <!-- front left leg -->
                @(taghead_body(link_name="Lfl1_hip_roll"))
                    @(tag_joint(joint_name="Jfl1_hip_roll", indent=5))
                    @(tag_inertial(link_name="Lfl1_hip_roll"))
                    <geom class="viscol" mesh="mesh_lfl1_hip_roll" />

                    @(taghead_body(link_name="Lfl2_hip_pitch"))
                        @(tag_joint(joint_name="Jfl2_hip_pitch"))
                        @(tag_inertial(link_name="Lfl2_hip_pitch"))
                        <geom class="viscol" mesh="mesh_lfl2_hip_pitch" />

                        @(taghead_body(link_name="Lfl3_knee"))
                            @(tag_joint(joint_name="Jfl3_knee"))
                            @(tag_inertial(link_name="Lfl3_knee"))
                            <geom class="viscol" mesh="mesh_lfl3_knee" />

                            @(taghead_body(link_name="Lfl4_wheel"))
                                @(tag_joint(joint_name="Jfl4_wheel"))
                                @(tag_inertial(link_name="Lfl4_wheel"))
                                <geom class="rubber_wheel" mesh="mesh_lfl4_wheel" />
                            </body>
                        </body>
                    </body>
                </body>
                <!-- end of front left leg -->

                <!-- front right leg -->
                @(taghead_body(link_name="Lfr1_hip_roll", indent=4))
                    @(tag_joint(joint_name="Jfr1_hip_roll", indent=5))
                    @(tag_inertial(link_name="Lfr1_hip_roll", indent=5))
                    <geom class="viscol" mesh="mesh_lfr1_hip_roll" />

                    @(taghead_body(link_name="Lfr2_hip_pitch"))
                        @(tag_joint(joint_name="Jfr2_hip_pitch"))
                        @(tag_inertial(link_name="Lfr2_hip_pitch"))
                        <geom class="viscol" mesh="mesh_lfr2_hip_pitch" />

                        @(taghead_body(link_name="Lfr3_knee"))
                            @(tag_joint(joint_name="Jfr3_knee"))
                            @(tag_inertial(link_name="Lfr3_knee"))
                            <geom class="viscol" mesh="mesh_lfr3_knee" />

                            @(taghead_body(link_name="Lfr4_wheel"))
                                @(tag_joint(joint_name="Jfr4_wheel"))
                                @(tag_inertial(link_name="Lfr4_wheel"))
                                <geom class="rubber_wheel" mesh="mesh_lfr4_wheel" />
                            </body>
                        </body>
                    </body>
                </body>
                <!-- end of front right leg -->
                
                <!-- rear body -->
                @(taghead_body(link_name="L1_body_rear", indent=4))
                    @(tag_joint(joint_name="J1_waist", indent=5))
                    @(tag_inertial(link_name="L1_body_rear", indent=5))
                    <geom class="viscol" mesh="mesh_body_rear" />

                    <!-- rear left leg -->
                    @(taghead_body(link_name="Lrl1_hip_roll", indent=5))
                        @(tag_joint(joint_name="Jrl1_hip_roll", indent=6))
                        @(tag_inertial(link_name="Lrl1_hip_roll", indent=6))
                        <geom class="viscol" mesh="mesh_lrl1_hip_roll" />

                        @(taghead_body(link_name="Lrl2_hip_pitch"))
                            @(tag_joint(joint_name="Jrl2_hip_pitch"))
                            @(tag_inertial(link_name="Lrl2_hip_pitch"))
                            <geom class="viscol" mesh="mesh_lrl2_hip_pitch" />

                            @(taghead_body(link_name="Lrl3_knee"))
                                @(tag_joint(joint_name="Jrl3_knee"))
                                @(tag_inertial(link_name="Lrl3_knee"))
                                <geom class="viscol" mesh="mesh_lrl3_knee" />

                                @(taghead_body(link_name="Lrl4_wheel"))
                                    @(tag_joint(joint_name="Jrl4_wheel"))
                                    @(tag_inertial(link_name="Lrl4_wheel"))
                                    <geom class="rubber_wheel" mesh="mesh_lrl4_wheel" />
                                </body>
                            </body>
                        </body>
                    </body>
                    <!-- end of rear left leg -->

                    <!-- rear right leg -->
                    @(taghead_body(link_name="Lrr1_hip_roll", indent=5))
                        @(tag_joint(joint_name="Jrr1_hip_roll", indent=6))
                        @(tag_inertial(link_name="Lrr1_hip_roll", indent=6))
                        <geom class="viscol" mesh="mesh_lrr1_hip_roll" />

                        @(taghead_body(link_name="Lrr2_hip_pitch"))
                            @(tag_joint(joint_name="Jrr2_hip_pitch"))
                            @(tag_inertial(link_name="Lrr2_hip_pitch"))
                            <geom class="viscol" mesh="mesh_lrr2_hip_pitch" />

                            @(taghead_body(link_name="Lrr3_knee"))
                                @(tag_joint(joint_name="Jrr3_knee"))
                                @(tag_inertial(link_name="Lrr3_knee"))
                                <geom class="viscol" mesh="mesh_lrr3_knee" />

                                @(taghead_body(link_name="Lrr4_wheel"))
                                    @(tag_joint(joint_name="Jrr4_wheel"))
                                    @(tag_inertial(link_name="Lrr4_wheel"))
                                    <geom class="rubber_wheel" mesh="mesh_lrr4_wheel" />
                                </body>
                            </body>
                        </body>
                    </body>
                    <!-- end of rear right leg -->
                </body>
                <!-- end of rear body -->
            </body>
            <!-- end of front body -->
        </body>
        <!-- robot floating base (root) -->
    </worldbody>

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

    <actuator>
        <!-- front left leg -->
        @(tag_motor(actuator_name='Rfl1_hip_roll',  indent=2))
        @(tag_motor(actuator_name='Rfl2_hip_pitch', indent=2))
        @(tag_motor(actuator_name='Rfl3_knee',      indent=2))
        @(tag_motor(actuator_name='Rfl4_wheel',     indent=2))
        <!-- front right leg -->
        @(tag_motor(actuator_name='Rfr1_hip_roll',  indent=2))
        @(tag_motor(actuator_name='Rfr2_hip_pitch', indent=2))
        @(tag_motor(actuator_name='Rfr3_knee',      indent=2))
        @(tag_motor(actuator_name='Rfr4_wheel',     indent=2))
        <!-- rear left leg -->
        @(tag_motor(actuator_name='Rrl1_hip_roll',  indent=2))
        @(tag_motor(actuator_name='Rrl2_hip_pitch', indent=2))
        @(tag_motor(actuator_name='Rrl3_knee',      indent=2))
        @(tag_motor(actuator_name='Rrl4_wheel',     indent=2))
        <!-- rear right leg -->
        @(tag_motor(actuator_name='Rrr1_hip_roll',  indent=2))
        @(tag_motor(actuator_name='Rrr2_hip_pitch', indent=2))
        @(tag_motor(actuator_name='Rrr3_knee',      indent=2))
        @(tag_motor(actuator_name='Rrr4_wheel',     indent=2))
        <!-- waist -->
        @(tag_motor(actuator_name='R1_waist',       indent=2))
    </actuator>
    
    <sensor>
        <!-- joint position `q_J` -->
        <!-- front left leg -->
        <jointpos name="q_Jfl1_hip_roll"  joint="Jfl1_hip_roll"     noise="@_jpos_noise" />
        <jointpos name="q_Jfl2_hip_pitch" joint="Jfl2_hip_pitch"    noise="@_jpos_noise" />
        <jointpos name="q_Jfl3_knee"      joint="Jfl3_knee"         noise="@_jpos_noise" />
        <jointpos name="q_Jfl4_wheel"     joint="Jfl4_wheel" />

        <!-- front right leg -->
        <jointpos name="q_Jfr1_hip_roll"  joint="Jfr1_hip_roll"     noise="@_jpos_noise" />
        <jointpos name="q_Jfr2_hip_pitch" joint="Jfr2_hip_pitch"    noise="@_jpos_noise" />
        <jointpos name="q_Jfr3_knee"      joint="Jfr3_knee"         noise="@_jpos_noise" />
        <jointpos name="q_Jfr4_wheel"     joint="Jfr4_wheel" />

        <!-- rear left leg -->
        <jointpos name="q_Jrl1_hip_roll"  joint="Jrl1_hip_roll"     noise="@_jpos_noise" />
        <jointpos name="q_Jrl2_hip_pitch" joint="Jrl2_hip_pitch"    noise="@_jpos_noise" />
        <jointpos name="q_Jrl3_knee"      joint="Jrl3_knee"         noise="@_jpos_noise" />
        <jointpos name="q_Jrl4_wheel"     joint="Jrl4_wheel" />

        <!-- rear right leg -->
        <jointpos name="q_Jrr1_hip_roll"  joint="Jrr1_hip_roll"     noise="@_jpos_noise" />
        <jointpos name="q_Jrr2_hip_pitch" joint="Jrr2_hip_pitch"    noise="@_jpos_noise" />
        <jointpos name="q_Jrr3_knee"      joint="Jrr3_knee"         noise="@_jpos_noise" />
        <jointpos name="q_Jrr4_wheel"     joint="Jrr4_wheel" />

        <!-- waist -->
        <jointpos name="q_J1_waist"       joint="J1_waist"          noise="@_jpos_noise" />

        <!-- joint velocity `dq_J` -->
        <!-- front left leg -->
        <jointvel name="dq_Jfl1_hip_roll"  joint="Jfl1_hip_roll"    noise="@_jvel_noise" />
        <jointvel name="dq_Jfl2_hip_pitch" joint="Jfl2_hip_pitch"   noise="@_jvel_noise" />
        <jointvel name="dq_Jfl3_knee"      joint="Jfl3_knee"        noise="@_jvel_noise" />
        <jointvel name="dq_Jfl4_wheel"     joint="Jfl4_wheel"       noise="@_jvel_noise" />

        <!-- front right leg -->
        <jointvel name="dq_Jfr1_hip_roll"  joint="Jfr1_hip_roll"    noise="@_jvel_noise" />
        <jointvel name="dq_Jfr2_hip_pitch" joint="Jfr2_hip_pitch"   noise="@_jvel_noise" />
        <jointvel name="dq_Jfr3_knee"      joint="Jfr3_knee"        noise="@_jvel_noise" />
        <jointvel name="dq_Jfr4_wheel"     joint="Jfr4_wheel"       noise="@_jvel_noise" />

        <!-- rear left leg -->
        <jointvel name="dq_Jrl1_hip_roll"  joint="Jrl1_hip_roll"    noise="@_jvel_noise" />
        <jointvel name="dq_Jrl2_hip_pitch" joint="Jrl2_hip_pitch"   noise="@_jvel_noise" />
        <jointvel name="dq_Jrl3_knee"      joint="Jrl3_knee"        noise="@_jvel_noise" />
        <jointvel name="dq_Jrl4_wheel"     joint="Jrl4_wheel"       noise="@_jvel_noise" />

        <!-- rear right leg -->
        <jointvel name="dq_Jrr1_hip_roll"  joint="Jrr1_hip_roll"    noise="@_jvel_noise" />
        <jointvel name="dq_Jrr2_hip_pitch" joint="Jrr2_hip_pitch"   noise="@_jvel_noise" />
        <jointvel name="dq_Jrr3_knee"      joint="Jrr3_knee"        noise="@_jvel_noise" />
        <jointvel name="dq_Jrr4_wheel"     joint="Jrr4_wheel"       noise="@_jvel_noise" />

        <!-- waist -->
        <jointvel name="dq_J1_waist"       joint="J1_waist"         noise="@_jvel_noise" />

        <!-- joint torque `tau_J` -->
        <!-- front left leg -->
        <jointactuatorfrc name="tau_Jfl1_hip_roll"  joint="Jfl1_hip_roll"   noise="@_jtor_noise" />
        <jointactuatorfrc name="tau_Jfl2_hip_pitch" joint="Jfl2_hip_pitch"  noise="@_jtor_noise" />
        <jointactuatorfrc name="tau_Jfl3_knee"      joint="Jfl3_knee"       noise="@_jtor_noise" />
        <jointactuatorfrc name="tau_Jfl4_wheel"     joint="Jfl4_wheel"      noise="@_jtor_noise" />

        <!-- front right leg -->
        <jointactuatorfrc name="tau_Jfr1_hip_roll"  joint="Jfr1_hip_roll"   noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jfr2_hip_pitch" joint="Jfr2_hip_pitch"  noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jfr3_knee"      joint="Jfr3_knee"       noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jfr4_wheel"     joint="Jfr4_wheel"      noise="@_jtor_noise"/>

        <!-- rear left leg -->
        <jointactuatorfrc name="tau_Jrl1_hip_roll"  joint="Jrl1_hip_roll"   noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrl2_hip_pitch" joint="Jrl2_hip_pitch"  noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrl3_knee"      joint="Jrl3_knee"       noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrl4_wheel"     joint="Jrl4_wheel"      noise="@_jtor_noise"/>

        <!-- rear right leg -->
        <jointactuatorfrc name="tau_Jrr1_hip_roll"  joint="Jrr1_hip_roll"   noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrr2_hip_pitch" joint="Jrr2_hip_pitch"  noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrr3_knee"      joint="Jrr3_knee"       noise="@_jtor_noise"/>
        <jointactuatorfrc name="tau_Jrr4_wheel"     joint="Jrr4_wheel"      noise="@_jtor_noise"/>

        <!-- waist -->
        <jointactuatorfrc name="tau_J1_waist"       joint="J1_waist"        noise="@_jtor_noise"/>

        <!-- IMU -->
        <gyro           name="imu_gyro" site="imu" noise="@_gyro_noise" />
        <accelerometer  name="imu_accl" site="imu" noise="@_accl_noise" />
        <framequat      name="imu_quat" objtype="site" objname="imu" noise="@_orie_noise" />
        <framepos       name="gt_pos"   objtype="site" objname="imu" />
        <framelinvel    name="gt_vel"   objtype="site" objname="imu" />
    </sensor>

</mujoco>
