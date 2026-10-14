# Confronto del codice e significato delle modifiche: AgileX `foxy` → LIMO

Base: [agilexrobotics/limo_ros2, commit `2faf97f`](https://github.com/agilexrobotics/limo_ros2/tree/2faf97f82993dbf3422a435eeccc7949a1ff2958). Workspace: ramo `limo_env`, commit `c4ab5cc`. Confronto del 12 ottobre 2026.

I percorsi upstream sono confrontati con `workspace/src/limo_ros2/`. **`+` = riga aggiunta; `-` = riga eliminata; una sostituzione mostra entrambe.** In `@@ -A,B +C,D @@`, `A` e `C` sono le righe iniziali nel file upstream e locale; `B` e `D` sono le lunghezze dei blocchi.

[Diff completo di tutti i 423 file aggiunti o modificati](DIFF_FOXY_WORKSPACE.patch), inclusi sorgenti importati, documentazione e risorse binarie. I 55 file identici non producono diff.

## Come leggere il confronto e la sua storia

Sono confrontati due riferimenti diversi: il codice originale **AgileX `limo_ros2/foxy`**
e la storia dei rami **`limo_env` e `foxy` di `chullius17/Limo`**. Le modifiche
ai driver rispetto ad AgileX sono state sviluppate nella storia comune di questa
repository; il ramo attuale le conserva e prepara l'installazione sul robot.

Le spiegazioni accanto al codice incrociano [README_report.md](workspace/src/custom_start/README_report.md),
il [README del profilo robot](ros2_foxy_dev/limo/README.md) e i diff dei commit.
Il report ricostruisce il lavoro fino a [`1a62922`](https://github.com/chullius17/Limo/commit/1a6292266a32317e8e8f2398cb4744cb92f339c8) del 4 ottobre 2026;
il README del profilo robot documenta l'installazione di questo ramo.
Le misure riportate sono quelle dei documenti storici, non nuove prove eseguite qui.

I due rami condividono [`1a62922`](https://github.com/chullius17/Limo/commit/1a6292266a32317e8e8f2398cb4744cb92f339c8). Dopo quel punto, `limo_env` ha **7 commit**
dedicati alla semplificazione e al setup; `foxy` ha **16 commit** successivi,
fino a [`d025baa`](https://github.com/chullius17/Limo/commit/d025baac111b0125ebc04a570546b0a4c679cf54) del 9 ottobre, dedicati alla pipeline applicativa.
La sezione [Storia dei due rami](#storia-dei-due-rami-della-repository-locale)
spiega quali modifiche sono comuni e quali appartengono soltanto a uno dei rami.

## File AgileX modificati: significato e diff completo

### [workspace/src/limo_ros2/limo_base/CMakeLists.txt](workspace/src/limo_ros2/limo_base/CMakeLists.txt)

**Origine:** [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d), 1 ottobre 2026.

`rcl_interfaces` serve a costruire il `ParameterDescriptor` delle nuove scale
di sterzo. Il driver le dichiara `read_only`: una calibrazione diversa deve
essere scelta prima dell'avvio, evitando di cambiare la conversione dei comandi
durante il movimento. Le righe aggiunte al CMake rendono disponibile questa
interfaccia al target compilato; non modificano direttamente la cinematica.

````diff
diff --git a/workspace/src/limo_ros2/limo_base/CMakeLists.txt b/workspace/src/limo_ros2/limo_base/CMakeLists.txt
index c23a7ff..8f89d56 100644
--- a/workspace/src/limo_ros2/limo_base/CMakeLists.txt
+++ b/workspace/src/limo_ros2/limo_base/CMakeLists.txt
@@ -17,6 +17,7 @@ endif()
 
 find_package(ament_cmake REQUIRED)
 find_package(rclcpp REQUIRED)
+find_package(rcl_interfaces REQUIRED)
 find_package(std_msgs REQUIRED)
 find_package(geometry_msgs REQUIRED)
 find_package(nav_msgs REQUIRED)
@@ -44,7 +45,7 @@ add_executable(limo_base
     src/limo_base_node.cpp
     src/limo_driver.cpp
     src/serial_port.cpp)
-ament_target_dependencies(limo_base rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs limo_msgs)
+ament_target_dependencies(limo_base rclcpp rcl_interfaces std_msgs tf2 tf2_ros nav_msgs sensor_msgs limo_msgs)
 
 install(
   DIRECTORY include/
````

### [workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h](workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h)

**Origine:** [`b9dcd14`](https://github.com/chullius17/Limo/commit/b9dcd14f0ae194f9efe7861e107e221c77df7489), [`e892edc`](https://github.com/chullius17/Limo/commit/e892edcf87fffe6f36faf9d3bb900126c0104bb5) e [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d).

`imu_yaw_initialized_`, `last_theta_` e `real_theta_` mantengono il riferimento
e l'accumulatore dello yaw per ciascuna istanza, con valori iniziali definiti.
Sostituiscono lo stato inizializzato male e il flag globale della versione originale.
`motion_mode_override_` rappresenta la scelta della cinematica dei comandi.
Le scale sinistra/destra diventano membri configurabili anziché costanti:
il default generico resta 2.47, mentre il profilo del robot seleziona 1.0.

````diff
diff --git a/workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h b/workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h
index 8547b2e..7ef3dcf 100644
--- a/workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h
+++ b/workspace/src/limo_ros2/limo_base/include/limo_base/limo_driver.h
@@ -103,7 +103,9 @@ private:
 
     bool pub_odom_tf_ = false;
     bool use_mcnamu_ = false;
-    double present_theta_,last_theta_,delta_theta_,real_theta_,rad;
+    bool imu_yaw_initialized_ = false;
+    double last_theta_ = 0.0;
+    double real_theta_ = 0.0;
     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
     rclcpp::Publisher<limo_msgs::msg::LimoStatus>::SharedPtr status_publisher_;
 
@@ -125,12 +127,13 @@ private:
 
     ImuData imu_data_;
     uint8_t motion_mode_;  // current motion type
+    int64_t motion_mode_override_ = -1;  // -1: use the mode reported by the chassis
 
     static constexpr double max_inner_angle_ = 0.48869;  // 28 degree
     static constexpr double track_ = 0.172;           // m (left right wheel distance)
     static constexpr double wheelbase_ = 0.2;         // m (front rear wheel distance)
-    static constexpr double left_angle_scale_ = 2.47;
-    static constexpr double right_angle_scale_ = 2.47;
+    double left_angle_scale_ = 2.47;
+    double right_angle_scale_ = 2.47;
 };
 
 }
````

### [workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py](workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py)

**Origine:** [`717253c`](https://github.com/chullius17/Limo/commit/717253cdc478b0e159197a61f4022bda3d7ab03e) per il remapping IMU, [`b9dcd14`](https://github.com/chullius17/Limo/commit/b9dcd14f0ae194f9efe7861e107e221c77df7489) per la modalità,
[`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d) per le scale di sterzo.

`imu_topic_name` permette al bringup reale di pubblicare su `/limo/imu`, il topic
consumato dall'EKF. `motion_mode` consente di usare Ackermann anche se il feedback
del firmware indica Mecanum. Il launch generico usa `-1` per la scelta automatica
e scale 2.47; il launch reale gli passa modalità 1 e scale 1.0. I nuovi argomenti
rendono queste scelte accessibili senza cambiare il sorgente del driver.

````diff
diff --git a/workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py b/workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py
index 06084ee..73c4f6b 100755
--- a/workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py
+++ b/workspace/src/limo_ros2/limo_base/launch/limo_base.launch.py
@@ -21,8 +21,20 @@ def generate_launch_description():
                                                 description='Base link frame id')
     odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                            description='Odometry topic name')
+    imu_topic_arg = DeclareLaunchArgument('imu_topic_name', default_value='/imu',
+                                          description='IMU topic name')
     odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='True',
                                            description='Odometry topic name')
+    motion_mode_arg = DeclareLaunchArgument(
+        'motion_mode', default_value='-1',
+        description='Override chassis motion mode: -1 auto, 0 differential, 1 Ackermann, 2 Mecanum')
+
+    steering_args = [
+        DeclareLaunchArgument(
+            name, default_value='2.47',
+            description='Inner-wheel/protocol angle ratio; finite and >= 1.0.')
+        for name in ('steering_left_scale', 'steering_right_scale')
+    ]
 
     # is_scout_mini_arg = DeclareLaunchArgument('is_scout_mini', default_value='false',
     #                                       description='Scout mini model')
@@ -39,6 +51,9 @@ def generate_launch_description():
         executable='limo_base',  #foxy executable='limo_base',
         output='screen',
         emulate_tty=True,
+        remappings=[
+            ('/imu', launch.substitutions.LaunchConfiguration('imu_topic_name')),
+        ],
         parameters=[{
                 # 'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                 'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
@@ -49,7 +64,10 @@ def generate_launch_description():
                 # 'is_omni_wheel': launch.substitutions.LaunchConfiguration('is_omni_wheel'),
                 # 'simulated_robot': launch.substitutions.LaunchConfiguration('simulated_robot'),
                 'pub_odom_tf': launch.substitutions.LaunchConfiguration('pub_odom_tf'),
+                'motion_mode': launch.substitutions.LaunchConfiguration('motion_mode'),
                 'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
+                'steering_left_scale': LaunchConfiguration('steering_left_scale'),
+                'steering_right_scale': LaunchConfiguration('steering_right_scale'),
         }])
 
     return LaunchDescription([
@@ -58,7 +76,10 @@ def generate_launch_description():
         odom_frame_arg,
         base_link_frame_arg,
         odom_topic_arg,
+        imu_topic_arg,
         odom_tf_arg,
+        motion_mode_arg,
+        *steering_args,
         # is_scout_mini_arg,
         # is_omni_wheel_arg,
         # simulated_robot_arg,
````

### [workspace/src/limo_ros2/limo_base/package.xml](workspace/src/limo_ros2/limo_base/package.xml)

**Origine:** [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d).

La dipendenza `rcl_interfaces` corrisponde all'aggiunta nel CMake e all'uso
del descriptor dei parametri nel driver. Il manifest rende esplicita la
dipendenza per gli strumenti ROS e per l'installazione con rosdep.

````diff
diff --git a/workspace/src/limo_ros2/limo_base/package.xml b/workspace/src/limo_ros2/limo_base/package.xml
index 2483d74..545cdb8 100644
--- a/workspace/src/limo_ros2/limo_base/package.xml
+++ b/workspace/src/limo_ros2/limo_base/package.xml
@@ -10,6 +10,7 @@
   <buildtool_depend>ament_cmake</buildtool_depend>
   <build_depend>limo_msgs</build_depend>
   <depend>rclcpp</depend>
+  <depend>rcl_interfaces</depend>
   <depend>std_msgs</depend>
   <depend>tf2</depend>
   <depend>tf2_ros</depend>
````

### [workspace/src/limo_ros2/limo_base/src/limo_driver.cpp](workspace/src/limo_ros2/limo_base/src/limo_driver.cpp)

Il diff sotto contiene **tre correzioni funzionali distinte**.

**Modalità dei comandi — [`b9dcd14`](https://github.com/chullius17/Limo/commit/b9dcd14f0ae194f9efe7861e107e221c77df7489), 8 settembre.** Il telaio era stato
convertito meccanicamente ad Ackermann ma continuava a riportare modalità Mecanum.
`command_motion_mode` seleziona quindi la conversione di `/cmd_vel` usando il
parametro `motion_mode`, con ripiego sul feedback per valori fuori da `0..2`.
Il profilo reale sceglie 1. Le condizioni su `linear.x` e `angular.z` evitano
divisioni nei casi di arresto e marcia rettilinea. L'override non riprogramma
il firmware: `/limo_status` e il ramo di decodifica dell'odometria continuano
a usare `motion_mode_` ricevuto dal telaio.

**Integrazione yaw — [`e892edc`](https://github.com/chullius17/Limo/commit/e892edcf87fffe6f36faf9d3bb900126c0104bb5), 23 settembre.** Nel codice originale, le
variabili locali del primo campione nascondevano i membri e gli incrementi
inferiori a 0.1° venivano scartati. Una curva lenta composta da piccoli incrementi
poteva quindi sparire dall'odometria. Ora il primo campione imposta il riferimento,
l'accumulatore parte da zero e `std::remainder(delta, 360.0)` attraversa ±180°
senza un salto di quasi un giro. Lo yaw assoluto pubblicato nell'IMU continua
a essere calcolato dal suo campione; la modifica interessa l'accumulo odometrico.

**Calibrazione sterzo — [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d), 1 ottobre.** Il report odometrico registra
comandi di 0.5 m/s e ±1 rad/s, con risposta circa 0.421 / −0.460 rad/s e raggi
stimati di 1.18 / 1.08 m anziché 0.50 m. Il fattore fisso 2.47 riduceva troppo
l'angolo inviato per quel telaio. Esporre le scale e scegliere 1.0 nel profilo
reale aumenta l'angolo del protocollo per lo stesso comando. Il segno positivo
usa ora la scala sinistra, coerentemente con il feedback. La validazione rifiuta
valori non finiti o minori di 1; `read_only` impedisce cambi durante l'esecuzione.
Il limite della ruota interna resta 28° e il passo hardware resta 0.20 m.

I test seriali documentati verificano la conversione dei comandi e del feedback,
non l'esattezza della curva fisica. Il report lasciava ancora da ripetere una
prova di curva con la nuova scala. Fonte:
[report odometrico conservato su `foxy`](https://github.com/chullius17/Limo/blob/d025baac111b0125ebc04a570546b0a4c679cf54/limo_sim/workspace/src/custom_start/README_odometria_reale.md).

````diff
diff --git a/workspace/src/limo_ros2/limo_base/src/limo_driver.cpp b/workspace/src/limo_ros2/limo_base/src/limo_driver.cpp
index e8e9e05..1b23951 100644
--- a/workspace/src/limo_ros2/limo_base/src/limo_driver.cpp
+++ b/workspace/src/limo_ros2/limo_base/src/limo_driver.cpp
@@ -29,7 +29,9 @@
  */
 
 #include "limo_base/limo_driver.h"
-int flag=0; 
+#include <cmath>
+#include <stdexcept>
+#include "rcl_interfaces/msg/parameter_descriptor.hpp"
 
 namespace AgileX {
 
@@ -41,22 +43,48 @@ LimoDriver::LimoDriver(std::string node_name):rclcpp::Node(node_name),keep_runni
     this->declare_parameter("base_frame");
     this->declare_parameter("pub_odom_tf");
     this->declare_parameter("use_mcnamu");
+    this->declare_parameter("motion_mode");
     this->declare_parameter("control_rate");  
 
+    // Physical inner-wheel angle / protocol angle, for each signed turn.
+    // Keep startup-only: changing calibration during motion is discontinuous.
+    rcl_interfaces::msg::ParameterDescriptor steering_descriptor;
+    steering_descriptor.read_only = true;
+    steering_descriptor.description =
+        "Inner-wheel angle divided by protocol angle; finite and at least 1.0";
+    left_angle_scale_ = this->declare_parameter<double>(
+        "steering_left_scale", 2.47, steering_descriptor);
+    right_angle_scale_ = this->declare_parameter<double>(
+        "steering_right_scale", 2.47, steering_descriptor);
+    if (!std::isfinite(left_angle_scale_) || left_angle_scale_ < 1.0 ||
+        !std::isfinite(right_angle_scale_) || right_angle_scale_ < 1.0) {
+        throw std::invalid_argument("Steering scales must be finite and >= 1.0");
+    }
+    RCLCPP_INFO(this->get_logger(),
+                "Steering scales: left=%.3f, right=%.3f; inner-wheel limit=28 deg",
+                left_angle_scale_, right_angle_scale_);
+
     this->get_parameter_or<std::string>("port_name", port_name, "ttyTHS1");//获取参数
     this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
     this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
     this->get_parameter_or<bool>("pub_odom_tf", pub_odom_tf_, "false");
     this->get_parameter_or<bool>("use_mcnamu", use_mcnamu_, "false");
+    this->get_parameter_or<int64_t>("motion_mode", motion_mode_override_, -1);
 
     std::cout << "Loading parameters: " << std::endl;
     std::cout << "- port name: " << port_name << std::endl;
     std::cout << "- odom frame name: " << odom_frame_ << std::endl;
     std::cout << "- base frame name: " << base_frame_ << std::endl;
     std::cout << "- odom topic name: " << pub_odom_tf_ << std::endl;
+    if (motion_mode_override_ >= MODE_FOUR_DIFF &&
+        motion_mode_override_ <= MODE_MCNAMU) {
+        RCLCPP_INFO_STREAM(this->get_logger(),
+                           "Command motion mode override: " << motion_mode_override_);
+    }
         
     if(use_mcnamu_) {
         motion_mode_ = MODE_MCNAMU;
+        motion_mode_override_ = MODE_MCNAMU;
     }
     tf_broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
     odom_publisher_=this->create_publisher<nav_msgs::msg::Odometry>("/odom",50);
@@ -427,12 +455,31 @@ void LimoDriver::sendFrame(const LimoFrame& frame) {
 }
 void LimoDriver::twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
 {
-    switch (motion_mode_) {
+    // Keep motion_mode_ as raw chassis feedback, but allow the command
+    // conversion to be selected independently for mechanically converted
+    // robots whose firmware keeps reporting a stale mode.
+    int64_t configured_mode = -1;
+    this->get_parameter_or<int64_t>("motion_mode", configured_mode, -1);
+    const uint8_t command_motion_mode =
+        configured_mode >= MODE_FOUR_DIFF && configured_mode <= MODE_MCNAMU
+            ? static_cast<uint8_t>(configured_mode)
+            : motion_mode_;
+
+    switch (command_motion_mode) {
         case MODE_FOUR_DIFF: {
             setMotionCommand(msg->linear.x, msg->angular.z, 0, 0);
             break;
         }
         case MODE_ACKERMANN: {
+            if (std::fabs(msg->linear.x) < 1e-6) {
+                setMotionCommand(0, 0, 0, 0);
+                break;
+            }
+            if (std::fabs(msg->angular.z) < 1e-6) {
+                setMotionCommand(msg->linear.x, 0, 0, 0);
+                break;
+            }
+
             double r = msg->linear.x / msg->angular.z;
             if(fabs(r) < track_/2.0)
             {
@@ -450,8 +497,9 @@ void LimoDriver::twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg
             }
 
             double steering_angle;
+            // Positive steering is left, matching the feedback conversion.
             if (inner_angle > 0) {
-                steering_angle = inner_angle / right_angle_scale_;
+                steering_angle = inner_angle / left_angle_scale_;
             }
             else {
                 steering_angle = inner_angle / right_angle_scale_;
@@ -489,20 +537,18 @@ void LimoDriver::publishIMUData(double stamp) {
     tf2::Quaternion q;
     q.setRPY(0.0, 0.0, degToRad(imu_data_.yaw));
 
-    if (flag==0)
-    {
-        double present_theta_ =imu_data_.yaw;
-        double last_theta_ = imu_data_.yaw;
-        flag=1;    
-        
+    // Odometry yaw starts at zero at the first IMU sample. Keep state per
+    // driver instance, including before any IMU frame has been received.
+    if (!imu_yaw_initialized_) {
+        last_theta_ = imu_data_.yaw;
+        imu_yaw_initialized_ = true;
+    } else {
+        // Preserve slow turns and unwrap the IMU's +/-180 degree boundary.
+        // A per-sample deadband discards real motion at high sample rates.
+        const double delta_theta = std::remainder(imu_data_.yaw - last_theta_, 360.0);
+        real_theta_ += delta_theta;
+        last_theta_ = imu_data_.yaw;
     }
-    //ROS_INFO("flag:%d",flag);
-    present_theta_ = imu_data_.yaw;
-    delta_theta_ = present_theta_ - last_theta_;
-    if(delta_theta_< 0.1 && delta_theta_> -0.1) delta_theta_=0;
-    real_theta_ = real_theta_ + delta_theta_;
-    last_theta_ = present_theta_;
-    //ROS_INFO("present_theta_:%f;delta_theta_:%f;real_theta_:%f;last_theta_:%f",present_theta_,delta_theta_,real_theta_,last_theta_);
 
     imu_msg.orientation.x = q.x();
     imu_msg.orientation.y = q.y();
@@ -570,7 +616,7 @@ void LimoDriver::publishOdometry(double stamp, double linear_velocity,
         default:
             break;
     }
-    rad = degToRad(real_theta_);
+    const double rad = degToRad(real_theta_);
 
     position_x_ += cos(rad) * vx * dt - sin(rad) * vy * dt;
     position_y_ += sin(rad) * vx * dt + cos(rad) * vy * dt;
@@ -668,5 +714,3 @@ double LimoDriver::convertCentralAngleToInner(double central_angle) {
 }
 
 }
-
-
````

### [workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py](workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py)

**Origine:** [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030), 22 settembre 2026.

Il remapping separa i comandi manuali (`/cmd_vel_teleop`) da quelli autonomi.
Nello stesso commit viene introdotto `limo_controller/cmd_vel_mux.py`: ascolta
teleop e autonomia, assegna alla teleop priorità 100 contro 10 e pubblica il
comando selezionato su `/cmd_vel`, con timeout degli ingressi.

Nel ramo attuale [`f6689ed`](https://github.com/chullius17/Limo/commit/f6689ed31100250b26074f396a8c5b26e203dcbc) elimina il pacchetto che conteneva il mux,
ma conserva questo remapping. Il launch teleop, usato da solo con il bringup
attuale, non inoltra quindi i comandi al driver che ascolta `/cmd_vel`.
Installare il pacchetto `twist_mux` fra le dipendenze non avvia automaticamente
un nodo. Questa è una conseguenza della semplificazione del ramo.
L'argomento `repeat_rate` eliminato era dichiarato ma non passato al nodo;
copyright, docstring e pulizia degli import non cambiano la guida.

````diff
diff --git a/workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py b/workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py
index 2b94664..b17457c 100644
--- a/workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py
+++ b/workspace/src/limo_ros2/limo_bringup/launch/limon_teletop_keyboard.launch.py
@@ -1,27 +1,31 @@
-import os
-import sys
+# Copyright 2026 Giulio Cataldo
+#
+# Licensed under the Apache License, Version 2.0 (the "License");
+# you may not use this file except in compliance with the License.
+# You may obtain a copy of the License at
+#
+#     http://www.apache.org/licenses/LICENSE-2.0
+#
+# Unless required by applicable law or agreed to in writing, software
+# distributed under the License is distributed on an "AS IS" BASIS,
+# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
+# See the License for the specific language governing permissions and
+# limitations under the License.
+
+"""Launch keyboard teleoperation on the high-priority velocity channel."""
 
 import launch
 import launch_ros.actions
-from ament_index_python.packages import get_package_share_directory
-
 
 
 def generate_launch_description():
     ld = launch.LaunchDescription([
-        launch.actions.DeclareLaunchArgument(
-            name='repeat_rate',
-            default_value='50.0'
-        ),
         launch_ros.actions.Node(
             package='teleop_twist_keyboard',
             executable='teleop_twist_keyboard',
             name='teleop_twist',
-            # parameters=[
-            #     {
-            #         'repeat_rate': launch.substitutions.LaunchConfiguration('repeat_rate')
-            #     }
-            # ]
+            remappings=[('cmd_vel', '/cmd_vel_teleop')],
+            output='screen',
         )
     ])
     return ld
````

### [workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml](workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml)

**Origine:** [`0aa6c66`](https://github.com/chullius17/Limo/commit/0aa6c663481b24515d14d82ff93f44c1492d0eec), 24 settembre 2026.

`resolution_fixed` non era il nome letto dal driver: `fixed_resolution` attiva
la dimensione fissa dello scan. La dimensione viene stimata all'avvio e può
cambiare fra sessioni; questa correzione non dimostrava da sola un miglioramento
del mapping.

`invalid_range_is_inf: true` abilita la correzione aggiunta nello stesso commit
al driver: raggi mancanti o invalidi diventano `+inf` dopo il riempimento dei bin.
Nel rosbag diagnosticato circa il 42.12% dei raggi era zero. Il matcher SLAM
poteva trattarli come punti sull'origine del lidar, pur se RViz li nascondeva.
Nelle riproduzioni con loop closure disabilitata, la sola conversione degli
invalidi riduceva i salti oltre 25 cm da 5 a 0 e quelli oltre 5° da 3 a 0.
Restava necessaria la conferma in movimento sul robot; non furono applicate
nuove calibrazioni della TF laser. Fonte:
[report mapping su `foxy`](https://github.com/chullius17/Limo/blob/d025baac111b0125ebc04a570546b0a4c679cf54/limo_sim/workspace/src/custom_start/README_mapping_reale.md).

````diff
diff --git a/workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml b/workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml
index 45e1e7b..e552ed1 100644
--- a/workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml
+++ b/workspace/src/limo_ros2/limo_bringup/param/ydlidar.yaml
@@ -8,7 +8,8 @@ ydlidar_ros2_driver_node:
     device_type: 0
     sample_rate: 3
     abnormal_check_count: 4
-    resolution_fixed: true
+    # Keep scan geometry constant for SLAM Toolbox / Karto.
+    fixed_resolution: true
     reversion: true
     inverted: true
     auto_reconnect: true
@@ -20,4 +21,5 @@ ydlidar_ros2_driver_node:
     range_max: 12.0
     range_min: 0.01
     frequency: 8.0
-    invalid_range_is_inf: false
+    # Exclude missing returns from SLAM matching instead of creating zero-range points.
+    invalid_range_is_inf: true
````

### [workspace/src/limo_ros2/limo_bringup/readme.md](workspace/src/limo_ros2/limo_bringup/readme.md)

**Origine:** [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030).

Il comando documentato viene allineato al canale teleop introdotto insieme
al mux di priorità. Non è una correzione della cinematica: cambia il percorso
del messaggio nel grafo ROS. Il collegamento funzionava con il mux della
pipeline `foxy`; nel ramo attuale quel pacchetto è stato eliminato.

````diff
diff --git a/workspace/src/limo_ros2/limo_bringup/readme.md b/workspace/src/limo_ros2/limo_bringup/readme.md
index 0775c66..9712d5b 100644
--- a/workspace/src/limo_ros2/limo_bringup/readme.md
+++ b/workspace/src/limo_ros2/limo_bringup/readme.md
@@ -28,6 +28,6 @@ ros2 launch build_map_2d revo_build_map_2d.launch.py
 # 键盘控制
 
 ```shell
-ros2 run teleop_twist_keyboard teleop_twist_keyboard
+ros2 launch limo_bringup limon_teletop_keyboard.launch.py
 ros2 launch limo_bringup limo_start.launch.py
 ```
````

### [workspace/src/limo_ros2/readme.md](workspace/src/limo_ros2/readme.md)

**Origine:** [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030).

Il comando documentato viene allineato al canale teleop introdotto insieme
al mux di priorità. Non è una correzione della cinematica: cambia il percorso
del messaggio nel grafo ROS. Il collegamento funzionava con il mux della
pipeline `foxy`; nel ramo attuale quel pacchetto è stato eliminato.

````diff
diff --git a/workspace/src/limo_ros2/readme.md b/workspace/src/limo_ros2/readme.md
index c714ccc..6fb41fe 100644
--- a/workspace/src/limo_ros2/readme.md
+++ b/workspace/src/limo_ros2/readme.md
@@ -63,7 +63,8 @@ ros2 launch limo_base limo_base.launch.py
 Start the keyboard teleop node
 
 ```
-ros2 run teleop_twist_keyboard teleop_twist_keyboard
+ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
+  -r cmd_vel:=/cmd_vel_teleop
 ```
 
 # statement
@@ -79,4 +80,3 @@ The limo_car gazebo simulation function package is provided by us and the Instit
 
 
 
-
````

## Storia dei due rami della repository locale

Questo confronto aggiunge il contesto necessario per leggere le righe del diff
AgileX: un file aggiunto rispetto ad AgileX può essere già identico fra
`limo_env` e `foxy` di questa repository.

### Cosa è comune e cosa cambia nel codice di base

Confrontando i blob Git di `HEAD` con `origin/foxy`, con la corrispondenza
`workspace/` → `limo_sim/workspace/`:

- Tutti gli **86 file di `limo_ros2`** sono identici, inclusi driver, adattatore Gazebo, teleop e test.
- I **22 file operativi di `custom_start`** presenti qui sono identici; i README differiscono per disponibilità e storia documentale. Il mondo/launch ostacoli presente su `foxy` è stato eliminato da questo ramo.
- Tutti i **188 file dell'SDK YDLidar** e i **31 file del driver ROS lidar** sono identici.
- Nel driver Astra, **103 file sono identici**; la sola differenza di contenuto nei 104 file comuni è la rimozione dell'include inutilizzato da `ob_camera_info.cpp`, durante [`8624208`](https://github.com/chullius17/Limo/commit/8624208a118fb29b25153214994a631b64373152).

```diff
 #include <rclcpp/rclcpp.hpp>
-#include <nlohmann/json.hpp>
 #include <thread>
```

È una correzione di dipendenza di compilazione: non cambia intrinseci o
algoritmi della camera. Il README robot conserva il comando `sed` per applicarla,
ma nel checkout corrente l'include è già assente.

### I sette commit esclusivi di `limo_env`

| Commit | Significato nel risultato attuale |
| --- | --- |
| [`f6689ed`](https://github.com/chullius17/Limo/commit/f6689ed31100250b26074f396a8c5b26e203dcbc), 4 ottobre | Elimina 182 file dei pacchetti applicativi sotto `ros2_ws`, compresi CV, mapping semantico, controller/mux, traiettorie e app; elimina anche il mondo/launch ostacoli. Conserva chassis, sensori e bringup di base. |
| [`6233334`](https://github.com/chullius17/Limo/commit/6233334bc74afb373f81d39ccb78c6abaf26fde4), 4 ottobre | Elimina i 190 file dell'albero `navigation2` e report diagnostici; sposta gli script desktop in `asus`. I report restano consultabili nella storia e su `foxy`. |
| [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683), 4 ottobre | Introduce i profili WSL e installazione diretta del robot, con dipendenze, build, udev, camera opzionale e blocco dell'installazione Gazebo. |
| [`8624208`](https://github.com/chullius17/Limo/commit/8624208a118fb29b25153214994a631b64373152), 6 ottobre | Porta `workspace` e `ros2_foxy_dev` alla radice; adatta alcuni percorsi, aggiunge le impostazioni editor e conserva il report storico come `README_report.md`; rimuove l'include JSON inutilizzato della camera. |
| [`1d296de`](https://github.com/chullius17/Limo/commit/1d296de02cbfd2c2cadda3faafbe4a1004597a56), 6 ottobre | Introduce alias, setup shell e `--symlink-install`, rendendo più semplice build e avvio quotidiani. |
| [`f86a3e8`](https://github.com/chullius17/Limo/commit/f86a3e80cd29fffd897c930b93883fc4b89e965c), 6 ottobre | Aggiunge il README francese e `rqt_image_view`, aggiorna la guida camera e rimuove vecchi log/report. |
| [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a), 6 ottobre | Passa dalla build di copie in `.limo` alla compilazione diretta dei pacchetti selezionati in `workspace/src`, aggiornando install, wrapper di avvio e setup shell. |

Il risultato è un ambiente per preparare ed eseguire **hardware e odometria
filtrata**. Conservare o installare librerie Nav2/SLAM non ripristina i pacchetti
applicativi eliminati e non avvia automaticamente navigazione e mapping.
Il README robot cita ancora cinque directory di `ros2_ws` con sole cache;
nel workspace corrente quell'albero non esiste.

### I sedici commit successivi sul ramo `foxy`

Questi commit non sono antenati del ramo attuale. Il loro lavoro interessa
la pipeline che qui è stata eliminata; le correzioni ai driver sopra rimangono
comuni ai due rami.

| Commit/gruppo | Significato |
| --- | --- |
| [`6dfea0f`](https://github.com/chullius17/Limo/commit/6dfea0fdc60dc7920ef0627228eb90b9961e2d0d), 5 ottobre | Introduce telemetria e visualizzazione delle predizioni MPC, con messaggi/interfacce, preview e aggiornamenti del controllo. |
| [`bcff589`](https://github.com/chullius17/Limo/commit/bcff589117d7fcc7787ee3b389da970cc4a5de5e), 5 ottobre | Aggiunge il planner con posa iniziale esplicita e raccordo Dubins iniziale, collegando planner, traiettoria e controller. |
| [`0405038`](https://github.com/chullius17/Limo/commit/0405038081ab8911ac35b70cdf7627e83adb3a2e), [`8ff7423`](https://github.com/chullius17/Limo/commit/8ff7423ef0846938120e6303d3ffab83caad9e18), [`b9ea7e2`](https://github.com/chullius17/Limo/commit/b9ea7e2216ab30fda955b89970dc6c4a5cdfe597), [`39f8dbf`](https://github.com/chullius17/Limo/commit/39f8dbfeaccb97395845d594596608f8d2e4dc71), 5 ottobre | Riordina configurazioni/interfacce e ritocca controllo, mappa locale e traiettorie. |
| [`a26c6a0`](https://github.com/chullius17/Limo/commit/a26c6a04b8533672183ea7dd6bd587faa0a8630d), 7 ottobre | Modifica controller MPC, costi, inflation e mapping locale dopo le prove che evidenziavano movimenti a scatti e ostacoli non sempre evitati. |
| [`5ea4907`](https://github.com/chullius17/Limo/commit/5ea490768c788d097d6bd8e3f3168b1711761434), [`6f87128`](https://github.com/chullius17/Limo/commit/6f87128a3b2c453bca9915999f17906527638cd2), [`80e7412`](https://github.com/chullius17/Limo/commit/80e741214ba8d06b4dc2ed53b0167037f5fa58fe), [`d1db74d`](https://github.com/chullius17/Limo/commit/d1db74da757ad7478784c27d9d9ea9d04d089994), [`a61e7aa`](https://github.com/chullius17/Limo/commit/a61e7aa8647376b3e1bdf72bc64825366b657417), 7 ottobre | Affina GUI/RViz, profili e footprint, separando impostazioni per simulazione e robot e il loro budget di calcolo. |
| [`34bf287`](https://github.com/chullius17/Limo/commit/34bf28722c3384b2d47bc9c94ca5c48fa55477d7), 9 ottobre | Aggiorna parametri di simulazione, controllo e app dopo il lavoro di integrazione; il titolo del commit non sostituisce una validazione fisica. |
| [`58a685e`](https://github.com/chullius17/Limo/commit/58a685e1915bd4f23a24b70cf708ff6812dad2ec), 9 ottobre | Mantiene i controlli di collisione nel raccordo iniziale, riallinea il footprint fisico e tratta il costo semantico del marciapiede come penalità morbida anziché collisione obbligatoria; i ritorni laser restano ostacoli letali. |
| [`6d8d9c8`](https://github.com/chullius17/Limo/commit/6d8d9c844f965939d2e7a25a2d085ae91e834de5), 9 ottobre | Ritocca i profili reali di CV, controllo, mappa locale e traiettoria. |
| [`d025baa`](https://github.com/chullius17/Limo/commit/d025baac111b0125ebc04a570546b0a4c679cf54), 9 ottobre | Introduce il profilo a bande discrete di profondità e aggiorna soglie/classificazione nella point cloud visiva. |

Il lavoro successivo su `foxy` non viene incorporato automaticamente da una
build di `limo_env`: i relativi sorgenti qui non sono presenti.

### Risultati documentati e limiti della loro interpretazione

Il report storico raccoglie build e prove di Gazebo, camera e seriale, oltre
a confronti di rosbag. Il report odometrico documenta il miglioramento dell'EKF;
quello mapping separa la correzione dei raggi invalidi dalle prove diagnostiche
su loop closure, offset lidar e carico CPU. Questi risultati riguardano le
sessioni e le registrazioni descritte, non una nuova validazione di questo checkout.

Il [README robot](ros2_foxy_dev/limo/README.md) dice che il setup non è stato
interamente verificato sul robot mentre venivano preparati gli script. Riporta
anche controlli specifici del runtime sul Jetson: risoluzione delle dipendenze
di `liborbbec.so` non equivale a verifica dello streaming o della navigazione.
Non c'è quindi contraddizione tra le vecchie prove dei driver e la necessità
di verificare l'installazione riproducibile del ramo attuale.

## File aggiunti: codice completo di launch, nodi, configurazioni e script locali

Questi file non esistono nella base AgileX. Il diff parte da `/dev/null`; tutte le righe sono aggiunte.

### [workspace/src/custom_start/launch/limo_real.launch.py](workspace/src/custom_start/launch/limo_real.launch.py)

**Aggiunto: righe 1–224.**

**Origine:** [`717253c`](https://github.com/chullius17/Limo/commit/717253cdc478b0e159197a61f4022bda3d7ab03e) per il bringup fisico, con correzioni successive
nei commit [`cd0525c`](https://github.com/chullius17/Limo/commit/cd0525ce95cadaf772c33e007e93480c72774876), [`b9dcd14`](https://github.com/chullius17/Limo/commit/b9dcd14f0ae194f9efe7861e107e221c77df7489), [`0aa6c66`](https://github.com/chullius17/Limo/commit/0aa6c663481b24515d14d82ff93f44c1492d0eec), [`ae814b0`](https://github.com/chullius17/Limo/commit/ae814b03a8c9e9d33539ac55cc1c0e4406a7e97c) e [`4ce8f01`](https://github.com/chullius17/Limo/commit/4ce8f01cd8b802f1f99b8305fd2186264a00c809).

Il launch costruisce il grafo di base del robot: driver chassis, lidar,
camera, trasformazioni dei sensori ed EKF. Rimappa l'IMU su `/limo/imu`, forza
la conversione dei comandi Ackermann e disabilita la TF odometrica del chassis:
`odom → base_link` viene pubblicata dall'EKF, evitando due publisher concorrenti.

La camera usa il profilo DaBai U3, con RGB/depth e CameraInfo rimappati sui topic
del progetto. Gli interruttori camera/lidar e RViz permettono avvii ridotti per
diagnosi. L'EKF carica prima la configurazione comune e poi l'override reale.
Questo launch avvia hardware e odometria filtrata; non avvia autonomamente
controller, percezione applicativa, mapping o Nav2.

````diff
diff --git a/workspace/src/custom_start/launch/limo_real.launch.py b/workspace/src/custom_start/launch/limo_real.launch.py
new file mode 100644
index 0000000..e92a98f
--- /dev/null
+++ b/workspace/src/custom_start/launch/limo_real.launch.py
@@ -0,0 +1,224 @@
+"""Start the physical LIMO robot with wheel odometry and EKF fusion."""
+
+import os
+
+from ament_index_python.packages import get_package_share_directory
+from launch import LaunchDescription
+from launch.actions import (
+    DeclareLaunchArgument,
+    GroupAction,
+    IncludeLaunchDescription,
+    OpaqueFunction,
+)
+from launch.conditions import IfCondition
+from launch.launch_description_sources import (
+    AnyLaunchDescriptionSource,
+    PythonLaunchDescriptionSource,
+)
+from launch.substitutions import PathJoinSubstitution
+from launch_ros.actions import Node, SetRemap
+from launch_ros.substitutions import FindPackageShare
+
+from custom_start.launch_config import load_profile, scalar, setting
+
+
+def _launch_real(context):
+    profile = load_profile(context, ('launch', 'camera_driver'))
+    settings = profile['launch']
+
+    def value(name):
+        return setting(context, settings, name)
+
+    custom_start_share = get_package_share_directory('custom_start')
+    limo_base_share = get_package_share_directory('limo_base')
+
+    port_name = value('port_name')
+    use_lidar = value('use_lidar')
+    use_camera = value('use_camera')
+    camera_x = value('camera_x')
+    camera_y = value('camera_y')
+    camera_z = value('camera_z')
+    camera_roll = value('camera_roll')
+    camera_pitch = value('camera_pitch')
+    camera_yaw = value('camera_yaw')
+    open_rviz = value('open_rviz')
+
+    limo_base = IncludeLaunchDescription(
+        PythonLaunchDescriptionSource(
+            os.path.join(limo_base_share, 'launch', 'limo_base.launch.py')
+        ),
+        launch_arguments={
+            'port_name': port_name,
+            'odom_frame': 'odom',
+            'base_frame': 'base_link',
+            'odom_topic_name': 'odom',
+            'imu_topic_name': '/limo/imu',
+            # The EKF is the only publisher of odom -> base_link.
+            'pub_odom_tf': 'false',
+            # This chassis reports mode 2 even after its mechanical Ackermann
+            # conversion, so force the matching Twist-to-steering conversion.
+            'motion_mode': '1',
+            'steering_left_scale': value('steering_left_scale'),
+            'steering_right_scale': value('steering_right_scale'),
+        }.items(),
+    )
+
+    imu_transform = Node(
+        package='tf2_ros',
+        executable='static_transform_publisher',
+        name='base_link_to_imu',
+        arguments=[
+            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
+            'base_link', 'imu_link',
+        ],
+    )
+
+    lidar = IncludeLaunchDescription(
+        PythonLaunchDescriptionSource(
+            os.path.join(limo_base_share, 'launch', 'open_ydlidar_launch.py')
+        ),
+        condition=IfCondition(use_lidar),
+    )
+
+    camera_mount_transform = Node(
+        package='tf2_ros',
+        executable='static_transform_publisher',
+        name='base_link_to_camera',
+        condition=IfCondition(use_camera),
+        arguments=[
+            camera_x, camera_y, camera_z,
+            # Foxy uses the legacy positional order: yaw, pitch, roll.
+            camera_yaw, camera_pitch, camera_roll,
+            'base_link', 'camera_link',
+        ],
+    )
+
+    camera = GroupAction(
+        condition=IfCondition(use_camera),
+        actions=[
+            SetRemap(
+                src='/camera/color/image_raw',
+                dst='/rgb/image_raw',
+            ),
+            SetRemap(
+                src='/camera/color/camera_info',
+                dst='/rgb/camera_info',
+            ),
+            SetRemap(
+                src='/camera/depth/image_raw',
+                dst='/depth_camera/depth/image_raw',
+            ),
+            SetRemap(
+                src='/camera/depth/camera_info',
+                dst='/depth_camera/depth/camera_info',
+            ),
+            SetRemap(
+                src='/camera/depth/points',
+                dst='/depth/points',
+            ),
+            IncludeLaunchDescription(
+                AnyLaunchDescriptionSource(
+                    PathJoinSubstitution([
+                        FindPackageShare('astra_camera'),
+                        'launch',
+                        'dabai_u3.launch.xml',
+                    ])
+                ),
+                launch_arguments={
+                    name: scalar(profile['camera_driver'], name)
+                    for name in profile['camera_driver']
+                }.items(),
+            ),
+        ],
+    )
+
+    ekf = Node(
+        package='robot_localization',
+        executable='ekf_node',
+        name='ekf_filter_node',
+        # Keep the node-specific YAML overrides on the root namespace.
+        namespace='/',
+        output='screen',
+        parameters=[
+            os.path.join(custom_start_share, 'config', 'ekf.yaml'),
+            os.path.join(custom_start_share, 'config', 'ekf_real.yaml'),
+        ],
+    )
+
+    rviz = Node(
+        package='rviz2',
+        executable='rviz2',
+        name='rviz2',
+        output='screen',
+        condition=IfCondition(open_rviz),
+    )
+
+    return [
+        limo_base,
+        imu_transform,
+        lidar,
+        camera_mount_transform,
+        camera,
+        ekf,
+        rviz,
+    ]
+
+
+def generate_launch_description():
+    default_config = os.path.join(
+        get_package_share_directory('custom_start'),
+        'config', 'limo_real.yaml')
+    return LaunchDescription([
+        DeclareLaunchArgument(
+            'config_file', default_value=default_config,
+            description='YAML profile for the physical robot.'),
+        # 2026-10-01 cmd_vel/IMU trial indicates direct protocol steering.
+        # Expose overrides so this chassis calibration is reversible.
+        DeclareLaunchArgument(
+            'steering_left_scale', default_value='',
+            description='Left inner-wheel/protocol angle ratio (>= 1.0).'),
+        DeclareLaunchArgument(
+            'steering_right_scale', default_value='',
+            description='Right inner-wheel/protocol angle ratio (>= 1.0).'),
+        DeclareLaunchArgument(
+            'port_name', default_value='',
+            description='Serial device name used by the physical LIMO.',
+        ),
+        DeclareLaunchArgument(
+            'use_lidar', default_value='',
+            description='Start the physical YDLidar driver.',
+        ),
+        DeclareLaunchArgument(
+            'use_camera', default_value='',
+            description='Start the physical Astra depth camera.',
+        ),
+        DeclareLaunchArgument(
+            'camera_x', default_value='',
+            description='Camera X offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_y', default_value='',
+            description='Camera Y offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_z', default_value='',
+            description='Camera Z offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_roll', default_value='',
+            description='Camera roll relative to base_link in radians.',
+        ),
+        DeclareLaunchArgument(
+            'camera_pitch', default_value='',
+            description='Camera pitch relative to base_link in radians.',
+        ),
+        DeclareLaunchArgument(
+            'camera_yaw', default_value='',
+            description='Camera yaw relative to base_link in radians.',
+        ),
+        DeclareLaunchArgument(
+            'open_rviz', default_value='',
+            description='Start RViz.',
+        ),
+        OpaqueFunction(function=_launch_real),
+    ])
````

### [workspace/src/custom_start/launch/limo_circuit.launch.py](workspace/src/custom_start/launch/limo_circuit.launch.py)

**Aggiunto: righe 1–181.**

**Origine:** [`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2); allineamento della convenzione TF camera in [`4cc0076`](https://github.com/chullius17/Limo/commit/4cc0076d70470ede2872eff05f937a7b359c4d5b).

Il launch riunisce descrizione del robot, Gazebo, spawn, modelli del circuito
ed EKF in un unico avvio. La GUI è opzionale e il tempo è simulato. L'EKF
fornisce la TF odometrica anche in simulazione. La catena camera aggiunta usa
la convenzione del sensore reale; non rende identici intrinseci, risoluzioni
o geometria dei sensori. Gli offset del profilo configurano le TF aggiunte,
mentre i giunti dei sensori simulati restano definiti nello Xacro.

````diff
diff --git a/workspace/src/custom_start/launch/limo_circuit.launch.py b/workspace/src/custom_start/launch/limo_circuit.launch.py
new file mode 100644
index 0000000..a869d88
--- /dev/null
+++ b/workspace/src/custom_start/launch/limo_circuit.launch.py
@@ -0,0 +1,181 @@
+"""Launch Gazebo with the custom LIMO circuit and the Ackermann LIMO robot."""
+
+import os
+
+from ament_index_python.packages import get_package_share_directory
+from launch import LaunchDescription
+from launch.actions import (
+    DeclareLaunchArgument,
+    ExecuteProcess,
+    IncludeLaunchDescription,
+    OpaqueFunction,
+    SetEnvironmentVariable,
+)
+from launch.conditions import IfCondition
+from launch.launch_description_sources import PythonLaunchDescriptionSource
+from launch.substitutions import EnvironmentVariable
+from launch_ros.actions import Node
+
+from custom_start.launch_config import boolean, load_profile, scalar, setting
+
+
+def _launch_circuit(context):
+    profile = load_profile(context, ('launch', 'spawn'))
+    settings = profile['launch']
+
+    def value(name):
+        return setting(context, settings, name)
+
+    custom_start_share = get_package_share_directory('custom_start')
+    limo_car_share = get_package_share_directory('limo_car')
+
+    ekf_config = os.path.join(custom_start_share, 'config', 'ekf.yaml')
+    model_path = os.path.join(custom_start_share, 'models')
+
+    world = value('world')
+    if not os.path.isabs(world):
+        world = os.path.join(custom_start_share, world)
+    gui = value('gui')
+    use_sim_time = value('use_sim_time')
+    camera_x = value('camera_x')
+    camera_y = value('camera_y')
+    camera_z = value('camera_z')
+    camera_roll = value('camera_roll')
+    camera_pitch = value('camera_pitch')
+    camera_yaw = value('camera_yaw')
+
+    robot_state_publisher = IncludeLaunchDescription(
+        PythonLaunchDescriptionSource(
+            os.path.join(limo_car_share, 'launch', 'ackermann.launch.py')
+        ),
+        launch_arguments={'use_sim_time': use_sim_time}.items(),
+    )
+
+    # Reproduce the physical-camera TF chain used by limo_real.launch.py.
+    camera_mount_transform = Node(
+        package='tf2_ros',
+        executable='static_transform_publisher',
+        name='sim_base_link_to_camera',
+        arguments=[
+            camera_x, camera_y, camera_z,
+            # Foxy positional order: yaw, pitch, roll.
+            camera_yaw, camera_pitch, camera_roll,
+            'base_link', 'camera_link',
+        ],
+    )
+
+    depth_optical_transform = Node(
+        package='tf2_ros',
+        executable='static_transform_publisher',
+        name='sim_camera_to_depth_optical',
+        arguments=[
+            '0.0', '0.0', '0.0',
+            # Same optical convention used by the physical Astra driver:
+            # roll=-pi/2, pitch=0, yaw=-pi/2.
+            '-1.57079632679', '0.0', '-1.57079632679',
+            'camera_link', 'depth_camera_frame_optical',
+        ],
+    )
+
+    ekf_node = Node(
+        package='robot_localization',
+        executable='ekf_node',
+        name='ekf_filter_node',
+        output='screen',
+        parameters=[
+            ekf_config,
+            {'use_sim_time': boolean(use_sim_time, 'use_sim_time')},
+        ],
+    )
+
+    gazebo_server = ExecuteProcess(
+        cmd=[
+            'gzserver', '--verbose', world,
+            '-s', 'libgazebo_ros_init.so',
+            '-s', 'libgazebo_ros_factory.so',
+        ],
+        output='screen',
+    )
+    gazebo_client = ExecuteProcess(
+        cmd=['gzclient'],
+        condition=IfCondition(gui),
+        output='screen',
+    )
+
+    spawn_robot = Node(
+        package='gazebo_ros',
+        executable='spawn_entity.py',
+        arguments=[
+            '-topic', 'robot_description',
+            '-entity', 'limo',
+            '-x', scalar(profile['spawn'], 'x'),
+            '-y', scalar(profile['spawn'], 'y'),
+            '-z', scalar(profile['spawn'], 'z'),
+            '-Y', scalar(profile['spawn'], 'yaw'),
+        ],
+        output='screen',
+    )
+
+    return [
+        SetEnvironmentVariable(
+            'GAZEBO_MODEL_PATH',
+            [model_path, ':', EnvironmentVariable(
+                'GAZEBO_MODEL_PATH', default_value='')],
+        ),
+        robot_state_publisher,
+        camera_mount_transform,
+        depth_optical_transform,
+        gazebo_server,
+        gazebo_client,
+        spawn_robot,
+        ekf_node,
+    ]
+
+
+def generate_launch_description():
+    default_config = os.path.join(
+        get_package_share_directory('custom_start'),
+        'config', 'limo_circuit.yaml')
+    return LaunchDescription([
+        DeclareLaunchArgument(
+            'config_file', default_value=default_config,
+            description='YAML profile for the Gazebo circuit.'),
+        DeclareLaunchArgument(
+            'world', default_value='',
+            description=(
+                'Gazebo world path; relative paths use custom_start share.'),
+        ),
+        DeclareLaunchArgument(
+            'gui', default_value='',
+            description='Start the Gazebo graphical client.',
+        ),
+        DeclareLaunchArgument(
+            'use_sim_time', default_value='',
+            description='Use the clock published by Gazebo.',
+        ),
+        DeclareLaunchArgument(
+            'camera_x', default_value='',
+            description='Camera X offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_y', default_value='',
+            description='Camera Y offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_z', default_value='',
+            description='Camera Z offset from base_link in metres.',
+        ),
+        DeclareLaunchArgument(
+            'camera_roll', default_value='',
+            description='Camera roll relative to base_link in radians.',
+        ),
+        DeclareLaunchArgument(
+            'camera_pitch', default_value='',
+            description='Camera pitch relative to base_link in radians.',
+        ),
+        DeclareLaunchArgument(
+            'camera_yaw', default_value='',
+            description='Camera yaw relative to base_link in radians.',
+        ),
+        OpaqueFunction(function=_launch_circuit),
+    ])
````

### [workspace/src/custom_start/custom_start/launch_config.py](workspace/src/custom_start/custom_start/launch_config.py)

**Aggiunto: righe 1–48.**

**Origine:** [`ae814b0`](https://github.com/chullius17/Limo/commit/ae814b03a8c9e9d33539ac55cc1c0e4406a7e97c) e [`4ce8f01`](https://github.com/chullius17/Limo/commit/4ce8f01cd8b802f1f99b8305fd2186264a00c809).

L'helper separa i valori di configurazione dal codice dei launch: legge il
profilo YAML, controlla le sezioni richieste e converte scalari/bool per launch.
Un argomento CLI non vuoto prevale sulla sezione `launch` del profilo.
`scalar` permette di inoltrare al driver tutte le voci di `camera_driver`,
evitando di dover riscrivere il launch per ogni nuovo argomento camera.

````diff
diff --git a/workspace/src/custom_start/custom_start/launch_config.py b/workspace/src/custom_start/custom_start/launch_config.py
new file mode 100644
index 0000000..a5561a7
--- /dev/null
+++ b/workspace/src/custom_start/custom_start/launch_config.py
@@ -0,0 +1,48 @@
+"""Read launch settings from a YAML profile with optional CLI overrides."""
+
+import os
+
+import yaml
+from launch.substitutions import LaunchConfiguration
+
+
+def load_profile(context, sections):
+    """Load the YAML file selected by the config_file launch argument."""
+    path = os.path.expanduser(
+        LaunchConfiguration('config_file').perform(context))
+    with open(path, encoding='utf-8') as stream:
+        profile = yaml.safe_load(stream)
+    for section in sections:
+        if not isinstance(profile, dict) or not isinstance(
+                profile.get(section), dict):
+            raise ValueError(
+                '{}: missing YAML mapping {!r}'.format(path, section))
+    return profile
+
+
+def setting(context, settings, name):
+    """Return a launch-friendly scalar, giving the CLI value precedence."""
+    override = LaunchConfiguration(name).perform(context)
+    value = override if override != '' else settings[name]
+    if isinstance(value, bool):
+        return str(value).lower()
+    if value is None or not isinstance(value, (str, int, float)):
+        raise ValueError('{} must be a string, number or boolean'.format(name))
+    return str(value)
+
+
+def scalar(settings, name):
+    """Convert a profile-only scalar to the string expected by launch."""
+    value = settings[name]
+    if isinstance(value, bool):
+        return str(value).lower()
+    if value is None or not isinstance(value, (str, int, float)):
+        raise ValueError('{} must be a string, number or boolean'.format(name))
+    return str(value)
+
+
+def boolean(value, name):
+    """Parse a launch boolean and reject typos in YAML or CLI arguments."""
+    if value.lower() not in ('true', 'false'):
+        raise ValueError('{} must be true or false'.format(name))
+    return value.lower() == 'true'
````

### [workspace/src/custom_start/config/limo_real.yaml](workspace/src/custom_start/config/limo_real.yaml)

**Aggiunto: righe 1–28.**

**Origine:** [`ae814b0`](https://github.com/chullius17/Limo/commit/ae814b03a8c9e9d33539ac55cc1c0e4406a7e97c) e [`4ce8f01`](https://github.com/chullius17/Limo/commit/4ce8f01cd8b802f1f99b8305fd2186264a00c809); le scelte hardware provengono
anche da [`cd0525c`](https://github.com/chullius17/Limo/commit/cd0525ce95cadaf772c33e007e93480c72774876), [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d), [`0e500b0`](https://github.com/chullius17/Limo/commit/0e500b0d5978db81caa0fd1b80ab808fcc853b21), [`a82139f`](https://github.com/chullius17/Limo/commit/a82139fd00432881494542fa06d9e0f9075adae0) e [`14e4fc0`](https://github.com/chullius17/Limo/commit/14e4fc0d48b246cd760f6fbe1af8580dde523ffc).

Le scale 1.0 sono la calibrazione scelta per il telaio; il default del driver
generico resta 2.47. RGB 640×480 e depth 640×400 corrispondono al profilo DaBai U3.
La sincronizzazione riguarda i tempi di acquisizione, non la registrazione
geometrica depth → RGB. IR e point cloud del driver sono disattivati: per questo
`/depth/points` non è atteso nell'avvio predefinito. `tf_publish_rate: 0.0`
pubblica le trasformazioni rigide su `/tf_static`, evitando errori di
estrapolazione temporale. RViz è disabilitato nel profilo reale.

````diff
diff --git a/workspace/src/custom_start/config/limo_real.yaml b/workspace/src/custom_start/config/limo_real.yaml
new file mode 100644
index 0000000..9c0f446
--- /dev/null
+++ b/workspace/src/custom_start/config/limo_real.yaml
@@ -0,0 +1,28 @@
+# Valori predefiniti di limo_real.launch.py. Gli argomenti ros2 launch
+# specificati da riga di comando hanno precedenza sulla sezione launch.
+launch:
+  steering_left_scale: 1.0
+  steering_right_scale: 1.0
+  port_name: ttyTHS1
+  use_lidar: true
+  use_camera: true
+  camera_x: 0.10             # metri rispetto a base_link
+  camera_y: 0.0
+  camera_z: 0.065
+  camera_roll: 0.0          # radianti
+  camera_pitch: 0.0
+  camera_yaw: 0.0
+  open_rviz: false
+
+# Argomenti del driver Astra.
+camera_driver:
+  camera_name: camera
+  color_width: 640
+  color_height: 480
+  depth_width: 640
+  depth_height: 400
+  enable_point_cloud: false
+  enable_ir: false
+  color_depth_synchronization: true
+  # <=0 pubblica le TF ottiche una sola volta su /tf_static.
+  tf_publish_rate: 0.0
````

### [workspace/src/custom_start/config/limo_circuit.yaml](workspace/src/custom_start/config/limo_circuit.yaml)

**Aggiunto: righe 1–19.**

**Origine:** [`ae814b0`](https://github.com/chullius17/Limo/commit/ae814b03a8c9e9d33539ac55cc1c0e4406a7e97c).

Raccoglie mondo, GUI, clock simulato, TF camera e posa iniziale dello spawn.
Cambiare questo file o selezionare un altro `config_file` evita modifiche al
launch Python. Il profilo del circuito e quello del robot hanno impostazioni
distinte: selezionare il clock simulato non calibra automaticamente i sensori.

````diff
diff --git a/workspace/src/custom_start/config/limo_circuit.yaml b/workspace/src/custom_start/config/limo_circuit.yaml
new file mode 100644
index 0000000..173194d
--- /dev/null
+++ b/workspace/src/custom_start/config/limo_circuit.yaml
@@ -0,0 +1,19 @@
+# Valori predefiniti di limo_circuit.launch.py. Per world si può usare
+# un percorso assoluto oppure un percorso relativo alla share di custom_start.
+launch:
+  world: worlds/limo_circuit_world.world
+  gui: true
+  use_sim_time: true
+  camera_x: 0.10             # metri rispetto a base_link
+  camera_y: 0.0
+  camera_z: 0.065
+  camera_roll: 0.0          # radianti
+  camera_pitch: 0.0
+  camera_yaw: 0.0
+
+# Posizione iniziale del robot in Gazebo (metri e radianti).
+spawn:
+  x: 0.0
+  y: 0.0
+  z: 0.30
+  yaw: 0.0
````

### [workspace/src/custom_start/config/ekf_real.yaml](workspace/src/custom_start/config/ekf_real.yaml)

**Aggiunto: righe 1–11.**

**Origine della correzione:** [`0aa6c66`](https://github.com/chullius17/Limo/commit/0aa6c663481b24515d14d82ff93f44c1492d0eec); trasferimento dell'override dal
launch al YAML in [`ae814b0`](https://github.com/chullius17/Limo/commit/ae814b03a8c9e9d33539ac55cc1c0e4406a7e97c).

Durante le curve l'odometria del telaio riportava yaw rate zero, mentre l'IMU
misurava rotazione. Le covarianze odometriche delle velocità erano inoltre zero.
Fondere i due ingressi contraddittori causava correzioni dello yaw anche dopo
l'arresto. Il dodicesimo elemento di `odom0_config`, indice 11, disabilita
soltanto lo yaw rate dell'odometria: l'IMU continua a fornirlo e lo yaw della
posa odometrica resta abilitato. `use_sim_time: false` seleziona il tempo reale.

Il report odometrico documenta una variazione massima dello yaw durante le
soste ridotta da 5.83° a 0.21° con l'override attivo. Il namespace radice nel
launch assicurava l'applicazione dell'override su Foxy. Questa correzione non
ripara il campo angolare o le covarianze del driver e non dimostra da sola
che il mapping sia risolto.

````diff
diff --git a/workspace/src/custom_start/config/ekf_real.yaml b/workspace/src/custom_start/config/ekf_real.yaml
new file mode 100644
index 0000000..31e58a5
--- /dev/null
+++ b/workspace/src/custom_start/config/ekf_real.yaml
@@ -0,0 +1,11 @@
+# Override dell'EKF per il LIMO fisico. La simulazione usa solo ekf.yaml.
+ekf_filter_node:
+  ros__parameters:
+    use_sim_time: false
+    # Il telaio reale riporta vyaw=0 anche in curva: lo yaw rate viene
+    # fuso soltanto da /limo/imu.
+    odom0_config: [true, true, false,
+                   false, false, true,
+                   true, true, false,
+                   false, false, false,
+                   false, false, false]
````

### [workspace/src/custom_start/config/ekf.yaml](workspace/src/custom_start/config/ekf.yaml)

**Aggiunto: righe 1–258.**

**Origine:** [`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2).

Il filtro planare combina odometria e yaw rate IMU a 100 Hz e pubblica
`/odometry/filtered` e `odom → base_link`. Il launch simulato usa questa base;
quello reale sovrappone `ekf_real.yaml`, mantenendo separata la correzione del
feedback hardware. La frequenza configurata non è una misura dell'effettiva
frequenza ottenuta sotto carico.

````diff
diff --git a/workspace/src/custom_start/config/ekf.yaml b/workspace/src/custom_start/config/ekf.yaml
new file mode 100644
index 0000000..5b4760d
--- /dev/null
+++ b/workspace/src/custom_start/config/ekf.yaml
@@ -0,0 +1,258 @@
+### ekf config file ###
+# SOURCE: https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml
+
+ekf_filter_node:
+    ros__parameters:
+        # The frequency, in Hz, at which the filter will output a position estimate. Note that the filter will not begin
+        # computation until it receives at least one message from one of the inputs. It will then run continuously at the
+        # frequency specified here, regardless of whether it receives more measurements. Defaults to 30 if unspecified.
+        frequency: 100.0
+
+        # The period, in seconds, after which we consider a sensor to have timed out. In this event, we carry out a predict
+        # cycle on the EKF without correcting it. This parameter can be thought of as the minimum frequency with which the
+        # filter will generate new output. Defaults to 1 / frequency if not specified.
+        sensor_timeout: 0.1
+
+        # ekf_localization_node and ukf_localization_node both use a 3D omnidirectional motion model. If this parameter is
+        # set to true, no 3D information will be used in your state estimate. Use this if you are operating in a planar
+        # environment and want to ignore the effect of small variations in the ground plane that might otherwise be detected
+        # by, for example, an IMU. Defaults to false if unspecified.
+        two_d_mode: true
+
+        # Use this parameter to provide an offset to the transform generated by ekf_localization_node. This can be used for
+        # future dating the transform, which is required for interaction with some other packages. Defaults to 0.0 if
+        # unspecified.
+        transform_time_offset: 0.0
+
+        # Use this parameter to provide specify how long the tf listener should wait for a transform to become available. 
+        # Defaults to 0.0 if unspecified.
+        transform_timeout: 0.0
+
+        # If you're having trouble, try setting this to true, and then echo the /diagnostics_agg topic to see if the node is
+        # unhappy with any settings or data.
+        print_diagnostics: true
+
+        # Debug settings. Not for the faint of heart. Outputs a ludicrous amount of information to the file specified by
+        # debug_out_file. I hope you like matrices! Please note that setting this to true will have strongly deleterious
+        # effects on the performance of the node. Defaults to false if unspecified.
+        debug: false
+
+        # Defaults to "robot_localization_debug.txt" if unspecified. Please specify the full path.
+        debug_out_file: /tmp/robot_localization_debug.txt
+
+        # Whether we'll allow old measurements to cause a re-publication of the updated state
+        permit_corrected_publication: false
+
+        # Whether to publish the acceleration state. Defaults to false if unspecified.
+        publish_acceleration: true
+        # ADDITION: I want to publish the acceleration state, maybe for control purposes
+
+        # Whether to broadcast the transformation over the /tf topic. Defaults to true if unspecified.
+        publish_tf: true
+
+        # REP-105 (http://www.ros.org/reps/rep-0105.html) specifies four principal coordinate frames: base_link, odom, map, and
+        # earth. base_link is the coordinate frame that is affixed to the robot. Both odom and map are world-fixed frames.
+        # The robot's position in the odom frame will drift over time, but is accurate in the short term and should be
+        # continuous. The odom frame is therefore the best frame for executing local motion plans. The map frame, like the odom
+        # frame, is a world-fixed coordinate frame, and while it contains the most globally accurate position estimate for your
+        # robot, it is subject to discrete jumps, e.g., due to the fusion of GPS data or a correction from a map-based
+        # localization node. The earth frame is used to relate multiple map frames by giving them a common reference frame.
+        # ekf_localization_node and ukf_localization_node are not concerned with the earth frame.
+        # Here is how to use the following settings:
+        # 1. Set the map_frame, odom_frame, and base_link frames to the appropriate frame names for your system.
+        #     1a. If your system does not have a map_frame, just remove it, and make sure "world_frame" is set to the value of
+        #         odom_frame.
+        # 2. If you are fusing continuous position data such as wheel encoder odometry, visual odometry, or IMU data, set
+        #   "world_frame" to your odom_frame value. This is the default behavior for robot_localization's state estimation nodes.
+        # 3. If you are fusing global absolute position data that is subject to discrete jumps (e.g., GPS or position updates
+        # from landmark observations) then:
+        #     3a. Set your "world_frame" to your map_frame value
+        #     3b. MAKE SURE something else is generating the odom->base_link transform. Note that this can even be another state
+        #         estimation node from robot_localization! However, that instance should *not* fuse the global data.
+        map_frame: map              # Defaults to "map" if unspecified
+        odom_frame: odom            # Defaults to "odom" if unspecified
+        base_link_frame: base_link  # Defaults to "base_link" if unspecified
+        world_frame: odom           # Defaults to the value of odom_frame if unspecified
+
+        # The filter accepts an arbitrary number of inputs from each input message type (nav_msgs/Odometry,
+        # geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/TwistWithCovarianceStamped,
+        # sensor_msgs/Imu). To add an input, simply append the next number in the sequence to its "base" name, e.g., odom0,
+        # odom1, twist0, twist1, imu0, imu1, imu2, etc. The value should be the topic name. These parameters obviously have no
+        # default values, and must be specified.
+        odom0: /odom
+
+        # Each sensor reading updates some or all of the filter's state. These options give you greater control over which
+        # values from each measurement are fed to the filter. For example, if you have an odometry message as input, but only
+        # want to use its Z position value, then set the entire vector to false, except for the third entry. The order of the
+        # values is x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az. Note that not some message types
+        # do not provide some of the state variables estimated by the filter. For example, a TwistWithCovarianceStamped message
+        # has no pose information, so the first six values would be meaningless in that case. Each vector defaults to all false
+        # if unspecified, effectively making this parameter required for each sensor.
+        odom0_config: [true, true, false,   # x, y, z
+                        false, false, true,  # roll, pitch, yaw
+                        true,  true,  false,  # vx, vy, vz
+                        false, false, true,   # vroll, vpitch, vyaw
+                        false, false, false]  # ax, ay, az
+
+        # If you have high-frequency data or are running with a low frequency parameter value, then you may want to increase
+        # the size of the subscription queue so that more measurements are fused.
+        odom0_queue_size: 10
+
+        # [ADVANCED] When measuring one pose variable with two sensors, a situation can arise in which both sensors under-
+        # report their covariances. This can lead to the filter rapidly jumping back and forth between each measurement as they
+        # arrive. In these cases, it often makes sense to (a) correct the measurement covariances, or (b) if velocity is also
+        # measured by one of the sensors, let one sensor measure pose, and the other velocity. However, doing (a) or (b) isn't
+        # always feasible, and so we expose the differential parameter. When differential mode is enabled, all absolute pose
+        # data is converted to velocity data by differentiating the absolute pose measurements. These velocities are then
+        # integrated as usual. NOTE: this only applies to sensors that provide pose measurements; setting differential to true
+        # for twist measurements has no effect.
+        odom0_differential: false
+
+        # [ADVANCED] When the node starts, if this parameter is true, then the first measurement is treated as a "zero point"
+        # for all future measurements. While you can achieve the same effect with the differential paremeter, the key
+        # difference is that the relative parameter doesn't cause the measurement to be converted to a velocity before
+        # integrating it. If you simply want your measurements to start at 0 for a given sensor, set this to true.
+        odom0_relative: false
+
+        # [ADVANCED] Whether to use the starting pose of child_frame_id as the origin of odometry.
+        # Note: this is different from setting odom0_relative to true, as when child_frame is different from 
+        # base_link_frame, the rotation of base_link will be coupled into the translation of child_frame.
+        # Set to true for fusing secondary odometry sources that are rigidly connected to base_link but has a non-zero 
+        # offset from base_link.
+        odom0_pose_use_child_frame: false
+
+        # [ADVANCED] If your data is subject to outliers, use these threshold settings, expressed as Mahalanobis distances, to
+        # control how far away from the current vehicle state a sensor measurement is permitted to be. Each defaults to
+        # numeric_limits<double>::max() if unspecified. It is strongly recommended that these parameters be removed if not
+        # required. Data is specified at the level of pose and twist variables, rather than for each variable in isolation.
+        # For messages that have both pose and twist data, the parameter specifies to which part of the message we are applying
+        # the thresholds.
+
+        # odom0_pose_rejection_threshold: 5.0
+        # odom0_twist_rejection_threshold: 1.0
+
+        # Further input parameter examples
+        # odom1: example/odom2
+        # odom1_config: [false, false, true,
+        #                false, false, false,
+        #                false, false, false,
+        #                false, false, true,
+        #                false, false, false]
+        # odom1_differential: false
+        # odom1_relative: true
+        # odom1_queue_size: 2
+        # odom1_pose_rejection_threshold: 2.0
+        # odom1_twist_rejection_threshold: 0.2
+        # pose0: example/pose
+        # pose0_config: [true,  true,  false,
+        #                false, false, false,
+        #                false, false, false,
+        #                false, false, false,
+        #                false, false, false]
+        # pose0_differential: true
+        # pose0_relative: false
+        # pose0_queue_size: 5
+        # pose0_rejection_threshold: 2.0  # Note the difference in parameter name
+
+        # twist0: example/twist
+        # twist0_config: [false, false, false,
+        #                 false, false, false,
+        #                 true,  true,  true,
+        #                 false, false, false,
+        #                 false, false, false]
+        # twist0_queue_size: 3
+        # twist0_rejection_threshold: 2.0
+
+        imu0: /limo/imu
+        imu0_config: [false, false, false,  # x, y, z
+                    false, false, false,  # roll, pitch, yaw
+                    false, false, false,  # vx, vy, vz
+                    false, false, true,   # vroll, vpitch, vyaw
+                    false, false, false]  # ax, ay, az
+
+        imu0_differential: false
+        imu0_relative: false
+        imu0_queue_size: 10
+
+        # imu0_pose_rejection_threshold: 0.8                 # Note the difference in parameter names
+        # imu0_twist_rejection_threshold: 0.8                #
+        # imu0_linear_acceleration_rejection_threshold: 0.8  #
+
+        # [ADVANCED] Some IMUs automatically remove acceleration due to gravity, and others don't. If yours doesn't, please set
+        # this to true, and *make sure* your data conforms to REP-103, specifically, that the data is in ENU frame.
+        imu0_remove_gravitational_acceleration: true
+        # ADDITION: We don't need it
+
+        # [ADVANCED]  The EKF and UKF models follow a standard predict/correct cycle. During prediction, if there is no
+        # acceleration reference, the velocity at time t+1 is simply predicted to be the same as the velocity at time t. During
+        # correction, this predicted value is fused with the measured value to produce the new velocity estimate. This can be
+        # problematic, as the final velocity will effectively be a weighted average of the old velocity and the new one. When
+        # this velocity is the integrated into a new pose, the result can be sluggish covergence. This effect is especially
+        # noticeable with LIDAR data during rotations. To get around it, users can try inflating the process_noise_covariance
+        # for the velocity variable in question, or decrease the  variance of the variable in question in the measurement
+        # itself. In addition, users can also take advantage of the control command being issued to the robot at the time we
+        # make the prediction. If control is used, it will get converted into an acceleration term, which will be used during
+        # predicition. Note that if an acceleration measurement for the variable in question is available from one of the
+        # inputs, the control term will be ignored.
+        # Whether or not we use the control input during predicition. Defaults to false.
+        use_control: false
+
+        # Whether the input (assumed to be cmd_vel) is a geometry_msgs/Twist or geometry_msgs/TwistStamped message. Defaults to
+        # false.
+        
+        # stamped_control: true
+
+        # The last issued control command will be used in prediction for this period. Defaults to 0.2.
+        control_timeout: 0.2
+
+        # Which velocities are being controlled. Order is vx, vy, vz, vroll, vpitch, vyaw.
+        control_config: [true, false, false, false, false, true]
+
+        # Places limits on how large the acceleration term will be. Should match your robot's kinematics.
+        acceleration_limits: [1.3, 0.0, 0.0, 0.0, 0.0, 3.4]
+
+        # Acceleration and deceleration limits are not always the same for robots.
+        deceleration_limits: [1.3, 0.0, 0.0, 0.0, 0.0, 4.5]
+
+        # If your robot cannot instantaneously reach its acceleration limit, the permitted change can be controlled with these
+        # gains
+        acceleration_gains: [0.8, 0.0, 0.0, 0.0, 0.0, 0.9]
+
+        # If your robot cannot instantaneously reach its deceleration limit, the permitted change can be controlled with these
+        # gains
+        deceleration_gains: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
+
+        # [ADVANCED] The process noise covariance matrix can be difficult to tune, and can vary for each application, so it is
+        # exposed as a configuration parameter. This matrix represents the noise we add to the total error after each
+        # prediction step. The better the omnidirectional motion model matches your system, the smaller these values can be.
+        # However, if users find that a given variable is slow to converge, one approach is to increase the
+        # process_noise_covariance diagonal value for the variable in question, which will cause the filter's predicted error
+        # to be larger, which will cause the filter to trust the incoming measurement more during correction. The values are
+        # ordered as x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az. Defaults to the matrix below if
+        # unspecified.
+        # Note: the specification of covariance matrices can be cumbersome, so all matrix parameters in this package support
+        # both full specification or specification of only the diagonal values.
+
+        # process_noise_covariance: [0.05, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.05, 0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.06, 0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.03, 0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.03, 0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.06, 0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025, 0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04, 0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01, 0.0,    0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01, 0.0,    0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.02, 0.0,    0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01, 0.0,    0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01, 0.0,
+        #                            0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]
+
+        # [ADVANCED] This represents the initial value for the state estimate error covariance matrix. Setting a diagonal
+        # value (variance) to a large value will result in rapid convergence for initial measurements of the variable in
+        # question. Users should take care not to use large values for variables that will not be measured directly. The values
+        # are ordered as x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az. Defaults to the diagonal values below
+        # if unspecified. In this example, we specify only the diagonal of the matrix.
+        
+        # initial_estimate_covariance: [1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9]
````

### [workspace/src/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py](workspace/src/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py)

**Aggiunto: righe 1–91.**

**Origine:** [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030), 22 settembre 2026.

L'applicazione usa `Twist.angular.z` come yaw rate in rad/s; il plugin Ackermann
Gazebo Foxy lo interpreta come angolo di sterzo. Inviare lo stesso numero ai
due sistemi produce quindi una curva diversa. L'adattatore calcola
`atan(wheelbase * yaw_rate / abs(speed))` e pubblica su `/cmd_vel_gazebo`.
Il valore assoluto tiene conto del segno che il plugin applica in retromarcia.

Saturazione, arresto per comandi non finiti o velocità quasi nulla e watchdog
da 0.5 s proteggono i casi limite. Il watchdog usa un clock stabile, così
continua a misurare il timeout anche quando il clock della simulazione non
progredisce. Il robot fisico continua a ricevere velocità e yaw rate su `/cmd_vel`.

````diff
diff --git a/workspace/src/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py b/workspace/src/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py
new file mode 100755
index 0000000..88eca60
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/scripts/gazebo_twist_adapter.py
@@ -0,0 +1,91 @@
+#!/usr/bin/env python3
+# Copyright 2026 Giulio Cataldo
+# Licensed under the Apache License, Version 2.0.
+
+"""Convert standard Twist yaw rate to Gazebo Foxy's steering-angle interface."""
+
+import math
+import time
+
+from geometry_msgs.msg import Twist
+import rclpy
+from rclpy.clock import Clock
+from rclpy.clock import ClockType
+from rclpy.node import Node
+
+
+def gazebo_command(velocity, yaw_rate, wheelbase, max_steering_angle):
+    """Return (speed, plugin angle), including Gazebo's reverse sign handling."""
+    if not math.isfinite(velocity) or not math.isfinite(yaw_rate):
+        return 0.0, 0.0
+    if abs(velocity) <= 0.001:
+        return 0.0, 0.0
+    # Gazebo internally multiplies angular.z by sign(linear.x). Its input is
+    # therefore atan(L*w/abs(v)), not standard yaw rate and not atan(L*w/v).
+    angle = math.atan(wheelbase * yaw_rate / abs(velocity))
+    return velocity, max(-max_steering_angle, min(max_steering_angle, angle))
+
+
+class GazeboTwistAdapter(Node):
+    """Keep /cmd_vel in SI units; publish the plugin-specific command separately."""
+
+    def __init__(self):
+        super().__init__('gazebo_twist_adapter')
+        self.declare_parameter('input_topic', '/cmd_vel')
+        self.declare_parameter('output_topic', '/cmd_vel_gazebo')
+        self.declare_parameter('wheelbase', 0.24)
+        # R >= 0.55 m leaves margin for the collision-center offsets in Gazebo.
+        self.declare_parameter('max_steering_angle', math.atan(0.24 / 0.55))
+        self.declare_parameter('command_timeout', 0.5)
+        self.wheelbase = self.get_parameter('wheelbase').value
+        self.max_angle = self.get_parameter('max_steering_angle').value
+        self.timeout = self.get_parameter('command_timeout').value
+        for value in (self.wheelbase, self.max_angle, self.timeout):
+            if not math.isfinite(value) or value <= 0.0:
+                raise ValueError('Adapter geometry and timeout must be finite and positive')
+        if self.max_angle >= math.pi / 2:
+            raise ValueError('max_steering_angle must be below pi/2')
+        input_topic = self.get_parameter('input_topic').value
+        output_topic = self.get_parameter('output_topic').value
+        if not input_topic or not output_topic or input_topic == output_topic:
+            raise ValueError('Input and output topics must be nonempty and different')
+        self.publisher = self.create_publisher(Twist, output_topic, 10)
+        self.subscription = self.create_subscription(Twist, input_topic, self._command, 10)
+        self.last_command_time = None
+        self.stop_sent = True
+        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
+        self.watchdog = self.create_timer(0.05, self._check_timeout, clock=self.watchdog_clock)
+
+    def _command(self, message):
+        command = Twist()
+        command.linear.x, command.angular.z = gazebo_command(
+            message.linear.x, message.angular.z, self.wheelbase, self.max_angle)
+        self.publisher.publish(command)
+        self.last_command_time = time.monotonic()
+        self.stop_sent = command.linear.x == 0.0 and command.angular.z == 0.0
+
+    def _check_timeout(self):
+        if (self.last_command_time is not None and not self.stop_sent
+                and time.monotonic() - self.last_command_time > self.timeout):
+            self.publisher.publish(Twist())
+            self.stop_sent = True
+
+
+def main(args=None):
+    """Run the simulation-only command converter."""
+    rclpy.init(args=args)
+    node = GazeboTwistAdapter()
+    try:
+        rclpy.spin(node)
+    except KeyboardInterrupt:
+        pass
+    finally:
+        if rclpy.ok():
+            node.publisher.publish(Twist())
+        node.destroy_node()
+        if rclpy.ok():
+            rclpy.shutdown()
+
+
+if __name__ == '__main__':
+    main()
````

### [workspace/src/limo_ros2/limo_car/gazebo/ackermann.xacro](workspace/src/limo_ros2/limo_car/gazebo/ackermann.xacro)

**Aggiunto: righe 1–389.**

**Origine:** [`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2), con aggiornamento del topic in [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030).

Il percorso Ackermann originale in `limo_description` richiamava plugin ROS 1.
Questo modello aggiunto usa `libgazebo_ros_ackermann_drive.so` e i giunti delle
quattro ruote e dello sterzo. Il remapping collega il plugin all'uscita
dell'adattatore, anziché ai comandi standard. La TF odometrica del plugin è
disabilitata per lasciarla all'EKF. Il modello/adattatore ha passo 0.24 m;
il driver fisico usa 0.20 m. Il vecchio `limo_description` resta invariato.

````diff
diff --git a/workspace/src/limo_ros2/limo_car/gazebo/ackermann.xacro b/workspace/src/limo_ros2/limo_car/gazebo/ackermann.xacro
new file mode 100644
index 0000000..55616b0
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/gazebo/ackermann.xacro
@@ -0,0 +1,389 @@
+<?xml version="1.0"?>
+<!--Auto: Zhui Li-->
+<!--E-Mail: lz554113510@gmail.com -->
+<!--Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig -->
+<!-- Description: Diese xacro-Datei basiert auf Regeln und definiert Funktionen durch xacro,
+um die Erstellung der Struktur des Wagens in Ackerman-Type zu ermöglichen. Es liegen die Karosserie des kleinen Wagens,
+die Rädern und Steuerscharnier im vorderen Achse, und auch die hinten Rädern. Das ganze System ist als eine Makrofunktion
+dargestellt. Es erhält sich noch die Funktionen über Reglung für Lenkung und Bewegung, wie die in Gazebo realisiert werden.
+Es würde einfach in andere Datei angerufen werden. -->
+
+<robot name="limo_ackermann" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <!-- ******************************** Paramenten zu definieren ************************************* -->
+    <!-- *********************************************************************************************** -->
+    <!-- definiert Value -->
+    <xacro:property name="M_PI" value="3.14159"/>
+
+    <!-- das Value sind all gemessen -->
+    <xacro:property name="base_mass" value="4.34"/>
+    <xacro:property name="base_x_size" value="0.19" />
+    <xacro:property name="base_y_size" value="0.31" />
+    <xacro:property name="base_z_size" value="0.12" />
+
+    <xacro:property name="wheelbase" value="0.24"/>
+    <xacro:property name="track" value="0.168"/>
+    <xacro:property name="wheel_vertical_offset" value="-0.10" />
+
+    <xacro:property name="wheel_mass" value="0.5" />
+    <xacro:property name="wheel_length" value="0.045" />
+    <xacro:property name="wheel_radius" value="0.045" />
+
+    <!-- definieren die Macro-Function für die Rechnungen der Matrizen für Trägheitsmoment-->
+    <!-- Zylinder -->
+    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
+        <inertial>
+            <origin xyz="0.0 0.0 0.0" />
+            <mass value="${m}" />
+            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
+                     iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
+                     izz="${m*r*r/2}" />
+        </inertial>
+    </xacro:macro>
+    <!-- Quader -->
+    <xacro:macro name="box_inertial_matrix" params="m w l h">
+        <inertial>
+            <origin xyz="0.0 0.0 0.0" />
+            <mass value="${m}" />
+            <inertia ixx="${m*(w*w+h*h)/12}" ixy = "0" ixz = "0"
+                     iyy="${m*(h*h+l*l)/12}" iyz = "0"
+                     izz="${m*(w*w+l*l)/12}" />
+        </inertial>
+    </xacro:macro>
+    <!-- *********************************************************************************************** -->
+    <!-- *********************************************************************************************** -->
+
+
+    <!-- **************************** Makrofunktion zu definieren ************************************** -->
+    <!-- *********************************************************************************************** -->
+    <!-- Makrofunktion für vorderen Räder mit Steuerscharnier-->
+    <!-- front left -->
+    <xacro:macro name="limo_left_steering_hinge" params="parent_prefix wheel_prefix *joint_pose">
+        <link name="${wheel_prefix}_steer_link">
+            <inertial>
+                <!-- diese Daten kommen aus den Herstellern -->
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <!-- es ist sehr klein, einfach als Klein definiert wird -->
+                    <cylinder length="0.0001" radius="0.0001" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius-0.01}" />
+                </geometry>
+            </collision>
+        </link>
+
+        <joint name="${wheel_prefix}_steer_joint" type="revolute">
+            <parent link="${parent_prefix}"/>
+            <child link="${wheel_prefix}_steer_link"/>
+            <xacro:insert_block name="joint_pose"/>
+            <axis xyz="0 0 1" />
+            <!-- Paramenten von Herstellern -->
+            <limit lower="-0.523598767" upper="0.523598767" effort="20" velocity="3.0" />
+            <dynamics damping="0.1" friction="0.1"/>
+        </joint>
+
+        <transmission name="${wheel_prefix}_hinge_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="${wheel_prefix}_hinge_motor">
+                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="${wheel_prefix}_steer_joint">
+                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+
+        <link name="${wheel_prefix}_wheel_link">
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="$(find limo_car)/meshes/limo_wheel.dae" scale="1.2 1.2 1.2"/>
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+            <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}"  h="${wheel_length}" />
+        </link>
+
+        <joint name="${wheel_prefix}_wheel_joint" type="continuous">
+            <parent link="${wheel_prefix}_steer_link"/>
+            <child link="${wheel_prefix}_wheel_link"/>
+            <origin xyz="0 0 0" rpy="0 0 0" />
+            <axis xyz="0 1 0"/>
+        </joint>
+
+        <transmission name="${wheel_prefix}_wheel_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="${wheel_prefix}_wheel_motor">
+                <!-- Type für den Topic(Paramenten) -->
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="${wheel_prefix}_wheel_joint">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+    </xacro:macro>
+
+    <!-- front right -->
+    <xacro:macro name="limo_right_steering_hinge" params="parent_prefix wheel_prefix *joint_pose">
+        <link name="${wheel_prefix}_steer_link">
+            <inertial>
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <cylinder length="0.0001" radius="0.0001" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius-0.01}" />
+                </geometry>
+            </collision>
+        </link>
+
+        <joint name="${wheel_prefix}_steer_joint" type="revolute">
+            <parent link="${parent_prefix}"/>
+            <child link="${wheel_prefix}_steer_link"/>
+            <xacro:insert_block name="joint_pose"/>
+            <axis xyz="0 0 -1"/>
+            <limit lower="-0.523598767" upper="0.523598767" effort="20" velocity="3.0" />
+            <dynamics damping="0.1" friction="0.1"/>
+        </joint>
+
+        <transmission name="${wheel_prefix}_hinge_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="${wheel_prefix}_hinge_motor">
+                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="${wheel_prefix}_steer_joint">
+                <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+
+        <link name="${wheel_prefix}_wheel_link">
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="$(find limo_car)/meshes/limo_wheel.dae" scale="1.2 1.2 1.2" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+            <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}"  h="${wheel_length}" />
+        </link>
+
+        <joint name="${wheel_prefix}_wheel_joint" type="continuous">
+            <parent link="${wheel_prefix}_steer_link"/>
+            <child link="${wheel_prefix}_wheel_link"/>
+            <origin xyz="0 0 0" rpy="0 0  0" />
+            <axis xyz="0 -1 0"/>
+        </joint>
+
+        <transmission name="${wheel_prefix}_wheel_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="${wheel_prefix}_wheel_motor">
+                <!-- Type für den Topic(Paramenten) -->
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="${wheel_prefix}_wheel_joint">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+    </xacro:macro>
+    <!-- *********************************************************************************************** -->
+    <!-- *********************************************************************************************** -->
+
+
+
+    <!-- ***************************************** Base zu bauen *************************************** -->
+    <!-- *********************************************************************************************** -->
+    <!-- erstellt Hauptanteil des Modells, definiert ein Nullpunkt, um Modell aufzubauen. Es ist einfach für weitere
+    Darstellungspostion in Simulation-Umgebung -->
+    <xacro:macro name="limo_ackermann">
+        <!-- <link name="base_footprint"/>
+
+        <joint name="base_joint" type="fixed">
+            <parent link="base_footprint"/>
+            <child link="base_link"/>
+            <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
+        </joint> -->
+
+        <!-- herstellt base_link -->
+        <link name="base_link">
+            <visual>
+                <origin xyz="0 0 -0.15" rpy="0 0 ${M_PI/2}" />
+                <geometry>
+                    <!-- Modell von Hersteller, aber es ist kleiner als realen Produkt, da die Skale vergrößert wird -->
+                    <mesh filename="$(find limo_car)/meshes/limo_base.dae" scale="1.2 1.2 1.2" />
+                </geometry>
+            </visual>
+            <!-- Modell für physikalische Kollision zu modellieren. -->
+            <collision>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <box size="${base_x_size} ${base_y_size} ${base_z_size}"/>
+                </geometry>
+            </collision>
+            <xacro:box_inertial_matrix  m="${base_mass}" w="${base_x_size}" l="${base_y_size}" h="${base_z_size}" />
+        </link>
+
+        <gazebo reference="base_link">
+            <material>
+                <color rgba="0 0 0 0.5"/>
+            </material>
+        </gazebo>
+
+        <!-- hinten Räder -->
+        <link name="rear_left_wheel_link">
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="$(find limo_car)/meshes/limo_wheel.dae" scale="1.2 1.2 1.2"/>
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+            <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}" h="${wheel_length}"/>
+        </link>
+
+        <joint name="rear_left_wheel_joint" type="continuous">
+            <parent link="base_link"/>
+            <child link="rear_left_wheel_link"/>
+            <origin xyz="${-wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
+            <axis xyz="0 1 0"/>
+        </joint>
+
+        <transmission name="rear_left_wheel_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="rear_left_wheel_motor">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="rear_left_wheel_joint">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+
+        <link name="rear_right_wheel_link">
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="$(find limo_car)/meshes/limo_wheel.dae" scale="1.2 1.2 1.2"/>
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="${M_PI/2} 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+            <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}" h="${wheel_length}"/>
+        </link>
+
+        <joint name="rear_right_wheel_joint" type="continuous">
+            <parent link="base_link"/>
+            <child link="rear_right_wheel_link"/>
+            <origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="${M_PI} 0 0" />
+            <axis xyz="0 -1 0"/>
+        </joint>
+
+        <transmission name="rear_right_wheel_trans" type="SimpleTransmission">
+            <type>transmission_interface/SimpleTransmission</type>
+            <actuator name="rear_right_wheel_motor">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+                <mechanicalReduction>1</mechanicalReduction>
+            </actuator>
+            <joint name="rear_right_wheel_joint">
+                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
+            </joint>
+        </transmission>
+
+        <!-- ruft Steuerscharnier mit vorderem Räder auf -->
+        <xacro:limo_left_steering_hinge parent_prefix="base_link" wheel_prefix="front_left">
+            <origin xyz="${wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
+        </xacro:limo_left_steering_hinge>
+
+        <xacro:limo_right_steering_hinge parent_prefix="base_link" wheel_prefix="front_right">
+            <origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="${M_PI} 0 0" />
+        </xacro:limo_right_steering_hinge>
+
+        <!-- ********************************** Kontrolle zu erstellen ************************************* -->
+        <!-- *********************************************************************************************** -->
+        <gazebo>
+            <plugin name="ackermann_controller" filename="libgazebo_ros_ackermann_drive.so">
+                <ros>
+                    <!-- Foxy subscribes to cmd_vel via ROS remapping; the
+                         legacy command_topic SDF tag is not read. -->
+                    <remapping>cmd_vel:=/cmd_vel_gazebo</remapping>
+                </ros>
+                <update_rate>100</update_rate>
+                <front_left_joint>front_left_wheel_joint</front_left_joint>
+                <front_right_joint>front_right_wheel_joint</front_right_joint>
+                <rear_left_joint>rear_left_wheel_joint</rear_left_joint>
+                <rear_right_joint>rear_right_wheel_joint</rear_right_joint>
+                <left_steering_joint>front_left_steer_joint</left_steering_joint>
+                <right_steering_joint>front_right_steer_joint</right_steering_joint>
+                <left_steering_pid_gain>10000 100 10</left_steering_pid_gain>
+                <left_steering_i_range>0 0</left_steering_i_range>
+                <right_steering_pid_gain>10000 100 10</right_steering_pid_gain>
+                <right_steering_i_range>0 0</right_steering_i_range>
+                <linear_velocity_pid_gain>0.5 10 0.001</linear_velocity_pid_gain>
+                <linear_velocity_i_range>0 0</linear_velocity_i_range>
+
+                <max_steer>0.523598767</max_steer> 
+
+                <wheel_separation>0.172</wheel_separation> 
+                <wheel_diameter>0.09</wheel_diameter> 
+                <max_wheel_torque>20</max_wheel_torque>
+                <max_wheel_acceleration>1.0</max_wheel_acceleration>
+                <publish_odom>true</publish_odom>
+                <publish_odom_tf>false</publish_odom_tf>        <!-- Set to false in order to use EKF node -->
+                <publish_wheel_tf>true</publish_wheel_tf>
+                <odometry_topic>odom</odometry_topic>
+                <odometry_frame>odom</odometry_frame>
+                <robot_base_frame>base_link</robot_base_frame>
+                <odometry_source>1</odometry_source>
+            </plugin>
+
+            <!--
+                Publish the measured steering joint positions for diagnostics.
+                Source: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp
+            -->
+            <plugin name="steering_joint_state_publisher"
+                    filename="libgazebo_ros_joint_state_publisher.so">
+                <update_rate>100</update_rate>
+                <joint_name>front_left_steer_joint</joint_name>
+                <joint_name>front_right_steer_joint</joint_name>
+            </plugin>
+        </gazebo>
+    </xacro:macro>
+</robot>
````

### [workspace/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro](workspace/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro)

**Aggiunto: righe 1–40.**

**Origine:** [`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2).

Assembla il modello Ackermann con lidar, camera e IMU e definisce i rispettivi
giunti di montaggio. Queste posizioni dello Xacro sono distinte dai parametri
delle TF camera aggiunte nel launch del circuito.

````diff
diff --git a/workspace/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro b/workspace/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro
new file mode 100644
index 0000000..df1c671
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/gazebo/ackermann_with_sensor.xacro
@@ -0,0 +1,40 @@
+<?xml version="1.0"?>
+<!--Auto: Zhui Li-->
+<!--E-Mail: lz554113510@gmail.com -->
+<!--Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig -->
+<!-- Description: Diese xacro-Datei basiert auf Regeln und definiert Funktionen durch xacro,
+um die Erstellung der ganzen Struktur des Wagens in Ackerman-Type zu ermöglichen. Das Fahrzeuggrundmodell und
+andere Sensoren zusammengebaut werden und laden sie das Modell in Gazebo.-->
+
+<robot name="limo_ackermann" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <xacro:include filename="$(find limo_car)/gazebo/ackermann.xacro" />
+    <xacro:include filename="$(find limo_car)/gazebo/sensor.xacro" />
+
+    <xacro:limo_ackermann />
+
+    <!-- ************************* Laser ****************************** -->
+    <xacro:gazebo_laser frame_prefix="laser" />
+
+    <joint name="laser_joint" type="fixed">
+        <origin xyz="0.1236 0 -0.01" rpy="0 0 0"/>
+        <parent link="base_link"/>
+        <child link="laser_link"/>
+    </joint>
+    <!-- ************************* Depth-camera *********************** -->
+    <xacro:gazebo_depth_camera frame_prefix="depth_camera" />
+
+    <joint name="depth_camera_joint" type="fixed">
+        <origin xyz="0.1 0 0.065" rpy="0 0 0"/>
+        <parent link="base_link"/>
+        <child link="depth_camera_link"/>
+    </joint>
+    <!-- ************************* IMU ******************************** -->
+    <xacro:gazebo_imu frame_prefix="imu" />
+
+    <joint name="imu_joint" type="fixed">
+        <origin xyz="0.0 0 -0.1" rpy="0 0 0"/>
+        <parent link="base_link"/>
+        <child link="imu_link"/>
+    </joint>
+</robot>
````

### [workspace/src/limo_ros2/limo_car/gazebo/sensor.xacro](workspace/src/limo_ros2/limo_car/gazebo/sensor.xacro)

**Aggiunto: righe 1–219.**

**Origine:** [`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2).

I plugin ROS 2 producono scan, immagini/depth, calibrazioni e IMU nel simulatore.
Il lidar simulato ha 720 campioni e circa 240° di campo; la camera è 320×240.
La pubblicazione IMU è collegata a `/limo/imu`, l'ingresso del filtro.
I nomi dei topic permettono di usare la stessa convenzione applicativa,
ma risoluzioni, campi visivi e geometria restano diversi dall'hardware.

````diff
diff --git a/workspace/src/limo_ros2/limo_car/gazebo/sensor.xacro b/workspace/src/limo_ros2/limo_car/gazebo/sensor.xacro
new file mode 100644
index 0000000..8a5bc65
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/gazebo/sensor.xacro
@@ -0,0 +1,219 @@
+<?xml version="1.0"?>
+<!-- Autor: Zhui Li -->
+<!--E-Mail: lz554113510@gmail.com -->
+<!--Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig -->
+<!-- Description: Diese xacro-Datei basiert auf Regeln und definiert Funktionen durch xacro,
+um die Erstellung der weitere Simulation für Sensoren in Gazebo ermöglichen. Es erhaltet die Makrofunktion für die
+realisierbar elektrische Komponente, also Sensoren aus Gazebo-Package. Diese Anteile konnte in anderen Datei benutzt werden. -->
+
+<robot name="limo_sensor" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <!-- ******************************* Makrofunktion für Laser-Sensor ***************************** -->
+    <xacro:macro name="gazebo_laser" params="frame_prefix">
+        <link name='${frame_prefix}_link'>
+            <inertial>
+                <mass value="0.1"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                        ixx="1e-6" ixy="0" ixz="0"
+                        iyy="1e-6" iyz="0"
+                        izz="1e-6"
+                />
+            </inertial>
+            <visual name='laser_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <cylinder radius="0.02" length="0.01"/>
+                </geometry>
+                <material name='laser_material'>
+                    <color rgba="1 1 1 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='laser_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <cylinder radius="0.032" length="0.016"/>
+                </geometry>
+            </collision>
+        </link>
+
+        <gazebo reference="${frame_prefix}_link">
+            <sensor type="ray" name="laser_sensor">
+                <visualize>false</visualize>
+                <update_rate>8</update_rate>
+                <ray>
+                    <scan>
+                        <horizontal>
+                            <samples>720</samples>
+                            <resolution>1</resolution>
+                            <min_angle>-2.09439504</min_angle> <!-- Einheit in "rad", von Hersteller -->
+                            <max_angle>2.09439504</max_angle>
+                        </horizontal>
+                    </scan>
+                    <range>
+                        <min>0.2</min>
+                        <max>8.0</max>
+                        <resolution>0.01</resolution>
+                    </range>
+                    <!-- um bessere Realität zu simulieren, wird Rauschen auch addiert -->
+                    <noise>
+                        <type>gaussian</type>
+                        <mean>0.0</mean>
+                        <stddev>0.01</stddev>
+                    </noise>
+                </ray>
+                <plugin name="gazebo_ros_laser_sensor" filename="libgazebo_ros_ray_sensor.so">
+                    <ros>
+                        <namespace>/</namespace>
+                        <remapping>~/out:=scan</remapping>
+                    </ros>
+                    <output_type>sensor_msgs/LaserScan</output_type>
+                    <min_intensity>150.0</min_intensity>
+                </plugin>
+            </sensor>
+        </gazebo>
+    </xacro:macro>
+
+    <!-- ******************************* Makrofunktion für Depth-Camera ***************************** -->
+    <xacro:macro name="gazebo_depth_camera" params="frame_prefix">
+        <link name='${frame_prefix}_link'>
+            <origin xyz="0 0 0" rpy="0 0 0"/>
+            <inertial>
+                <mass value="0.1"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                        ixx="1e-6" ixy="0" ixz="0"
+                        iyy="1e-6" iyz="0"
+                        izz="1e-6"
+                />
+            </inertial>
+            <visual name='depth_camera_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.02 0.06 0.015"/>
+                </geometry>
+                <material name='depth_camera_material'>
+                    <color rgba="0.75 0.75 0.75 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='depth_camera_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.02 0.06 0.015"/>
+                </geometry>
+            </collision>
+        </link>
+
+        <link name="depth_link" />
+
+        <joint name="${frame_prefix}_to_camera_joint" type="fixed">
+            <origin xyz="0 0 0" rpy="-1.570796 0 -1.570796 "/>
+            <parent link="${frame_prefix}_link"/>
+            <child link="depth_link"/>
+        </joint>
+
+        <gazebo reference="${frame_prefix}_link">
+            <sensor name="sensor_camera" type="depth">
+                <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
+                <always_on>true</always_on>
+                <visualize>true</visualize>
+                <update_rate>20.0</update_rate>
+                <camera name="head">
+                    <horizontal_fov>1.0471976</horizontal_fov> <!-- 60*pi/180 -->
+                    <!-- normalisieren die Form für Fotos -->
+                    <image>
+                        <width>320</width>
+                        <height>240</height>
+                        <format>R8G8B8</format>
+                    </image>
+                    <clip>
+                        <near>0.15</near>
+                        <far>8.0</far>
+                    </clip>
+                </camera>
+                <plugin name="gazebo_ros_depth_camera_sensor" filename="libgazebo_ros_camera.so">
+                    <ros>
+<!--                         <namespace>${frame_prefix}</namespace>-->
+                        <remapping>${frame_prefix}/image_raw:=rgb/image_raw</remapping>
+                        <remapping>${frame_prefix}/image_depth:=depth/image_raw</remapping>
+                        <remapping>${frame_prefix}/camera_info:=rgb/camera_info</remapping>
+                        <remapping>${frame_prefix}/camera_info_depth:=depth/camera_info</remapping>
+                        <remapping>${frame_prefix}/points:=depth/points</remapping>
+                    </ros>
+                    <camera_name>${frame_prefix}</camera_name>
+                    <frame_name>${frame_prefix}_frame_optical</frame_name>
+                    <hack_baseline>0.07</hack_baseline>
+                    <min_depth>0.001</min_depth>
+                    <max_depth>300.0</max_depth>
+                </plugin>
+            </sensor>
+        </gazebo>
+    </xacro:macro>
+
+    <!-- ******************************* Makrofunktion für IMU ***************************** -->
+    <xacro:macro name="gazebo_imu" params="frame_prefix">
+        <link name='${frame_prefix}_link'>
+            <pose>0 0 0 0 0 0</pose>
+            <inertial>
+                <mass value="0.01"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                        ixx="1e-7" ixy="0" ixz="0"
+                        iyy="1e-7" iyz="0"
+                        izz="1e-7"
+                />
+            </inertial>
+            <visual name='imu_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.001 0.001 0.001"/>
+                </geometry>
+                <material name='imu_material'>
+                    <color rgba="0.75 0.75 0.75 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='imu_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.001 0.001 0.001"/>
+                </geometry>
+            </collision>
+        </link>
+
+        <gazebo reference="${frame_prefix}_link">
+            <gravity>true</gravity>
+            <sensor name="imu_sensor" type="imu">
+                <always_on>true</always_on>
+                <update_rate>100</update_rate>
+                <visualize>true</visualize>
+                <topic>__default_topic__</topic>
+                <pose>0 0 0 0 0 0</pose>
+
+                <!--
+                    ROS 2 GazeboRosImuSensor publishes on ~/out; remap it to the
+                    topic consumed by robot_localization.
+                    Source: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp
+                -->
+                <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
+                    <ros>
+                        <remapping>~/out:=/limo/imu</remapping>
+                    </ros>
+                    <frame_name>${frame_prefix}_link</frame_name>
+                    <initial_orientation_as_reference>false</initial_orientation_as_reference>
+                </plugin>
+            </sensor>
+        </gazebo>
+    </xacro:macro>
+</robot>
````

### [.gitignore](.gitignore)

**Aggiunto: righe 1–24.**

````diff
diff --git a/.gitignore b/.gitignore
new file mode 100644
index 0000000..62e6b8d
--- /dev/null
+++ b/.gitignore
@@ -0,0 +1,24 @@
+build/
+install/
+log/
+
+# Runtime map exports; regenerated by the semantic map saver.
+/workspace/ros2_maps/**/*.pgm
+/workspace/ros2_maps/**/*.yaml
+
+# Python caches
+__pycache__/
+*.py[cod]
+*$py.class
+.pytest_cache/
+.mypy_cache/
+.ruff_cache/
+.cache/
+
+# Editor and IntelliSense caches
+browse.vc.db
+.vscode/ipch/
+
+.git-backups/
+Limo_robot_dev_mk2/control_logs/
+Limo_robot_dev_mk2/ai_control_logs/
````

### [ros2_foxy_dev/asus/Dockerfile](ros2_foxy_dev/asus/Dockerfile)

**Aggiunto: righe 1–68.**

**Origine:** ambiente iniziale [`16fa470`](https://github.com/chullius17/Limo/commit/16fa47006ef8aff2c3eb903d8bed9faa8282db60) e configurazione di simulazione
[`27b0c10`](https://github.com/chullius17/Limo/commit/27b0c108563eafffc8021d4e3ebd467560f675f2).

Ubuntu 20.04 e Foxy Desktop forniscono un ambiente di simulazione con Gazebo,
robot_localization e strumenti ROS. È un profilo desktop distinto dal setup
diretto del robot. La presenza delle librerie di percezione nell'immagine non
reintroduce i sorgenti applicativi eliminati dal ramo.

````diff
diff --git a/ros2_foxy_dev/asus/Dockerfile b/ros2_foxy_dev/asus/Dockerfile
new file mode 100644
index 0000000..73c9279
--- /dev/null
+++ b/ros2_foxy_dev/asus/Dockerfile
@@ -0,0 +1,68 @@
+FROM ubuntu:20.04
+
+ENV DEBIAN_FRONTEND=noninteractive
+
+# Set locale and install base development tools.
+RUN apt-get update && apt-get install -y --no-install-recommends \
+    locales \
+    curl \
+    gnupg2 \
+    lsb-release \
+    software-properties-common \
+    build-essential \
+    cmake \
+    git \
+    wget \
+    python3-pip \
+    python3-setuptools \
+    nano \
+    vim \
+    less \
+    tree \
+    && locale-gen en_US en_US.UTF-8 \
+    && rm -rf /var/lib/apt/lists/*
+
+ENV LANG=en_US.UTF-8
+ENV LC_ALL=en_US.UTF-8
+
+# Configure the ROS 2 repository for Ubuntu 20.04 (Focal).
+RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
+    -o /usr/share/keyrings/ros-archive-keyring.gpg \
+    && echo \
+    "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
+    > /etc/apt/sources.list.d/ros2.list
+
+# Install ROS 2 Foxy Desktop, Gazebo integration and workspace build tools.
+RUN apt-get update && apt-get install -y --no-install-recommends \
+    ros-foxy-desktop \
+    ros-foxy-gazebo-ros-pkgs \
+    ros-foxy-robot-localization \
+    ros-foxy-rqt-robot-steering \
+    ros-foxy-nav2-map-server \
+    ros-foxy-nav2-lifecycle-manager \
+    ros-foxy-xacro \
+    python3-rosdep \
+    python3-vcstool \
+    python3-colcon-common-extensions \
+    python3-argcomplete \
+    python3-opencv \
+    python3-scipy \
+    libopencv-dev \
+    libusb-1.0-0-dev \
+    libudev-dev \
+    libgl1-mesa-glx \
+    libglu1-mesa \
+    mesa-utils \
+    && rm -rf /var/lib/apt/lists/*
+
+# ONNX Runtime 1.10 is compatible with Foxy's Python 3.8 and the pipeline model.
+RUN python3 -m pip install --no-cache-dir onnxruntime==1.10.0
+
+# Initialize rosdep and automatically source Foxy in interactive shells.
+RUN rosdep init || true
+RUN rosdep update --rosdistro foxy
+RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
+
+WORKDIR /workspace
+
+CMD ["bash"]
````

### [ros2_foxy_dev/asus/build.sh](ros2_foxy_dev/asus/build.sh)

**Aggiunto: righe 1–6.**

````diff
diff --git a/ros2_foxy_dev/asus/build.sh b/ros2_foxy_dev/asus/build.sh
new file mode 100755
index 0000000..a4c1cf0
--- /dev/null
+++ b/ros2_foxy_dev/asus/build.sh
@@ -0,0 +1,6 @@
+#!/bin/bash
+set -e
+
+docker build \
+  -t limo_sim \
+  .
````

### [ros2_foxy_dev/asus/run.sh](ros2_foxy_dev/asus/run.sh)

**Aggiunto: righe 1–20.**

Il profilo originale abilita GPU, X11, rete e IPC host.
Il percorso `${SCRIPT_DIR}/../workspace` era coerente prima dello spostamento
degli script nella sottocartella `asus` in [`6233334`](https://github.com/chullius17/Limo/commit/6233334bc74afb373f81d39ccb78c6abaf26fde4). Nella struttura attuale
risolve a `ros2_foxy_dev/workspace`, che non esiste: il workspace presente è
`../../workspace` rispetto agli script. La correzione di mount documentata
nel report storico va quindi letta con il layout di quel momento.

````diff
diff --git a/ros2_foxy_dev/asus/run.sh b/ros2_foxy_dev/asus/run.sh
new file mode 100755
index 0000000..94d3568
--- /dev/null
+++ b/ros2_foxy_dev/asus/run.sh
@@ -0,0 +1,20 @@
+#!/bin/bash
+set -e
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+
+xhost +local:docker
+
+docker run \
+  -it \
+  --rm \
+  --gpus all \
+  --network host \
+  --ipc host \
+  -e DISPLAY="${DISPLAY}" \
+  -e QT_X11_NO_MITSHM=1 \
+  -e NVIDIA_VISIBLE_DEVICES=all \
+  -e NVIDIA_DRIVER_CAPABILITIES=all \
+  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
+  -v "${SCRIPT_DIR}/../workspace:/workspace" \
+  limo_sim
````

### [ros2_foxy_dev/asus/start_limo_docker.sh](ros2_foxy_dev/asus/start_limo_docker.sh)

**Aggiunto: righe 1–33.**

Il profilo originale abilita GPU, X11, rete e IPC host.
Il percorso `${SCRIPT_DIR}/../workspace` era coerente prima dello spostamento
degli script nella sottocartella `asus` in [`6233334`](https://github.com/chullius17/Limo/commit/6233334bc74afb373f81d39ccb78c6abaf26fde4). Nella struttura attuale
risolve a `ros2_foxy_dev/workspace`, che non esiste: il workspace presente è
`../../workspace` rispetto agli script. La correzione di mount documentata
nel report storico va quindi letta con il layout di quel momento.

````diff
diff --git a/ros2_foxy_dev/asus/start_limo_docker.sh b/ros2_foxy_dev/asus/start_limo_docker.sh
new file mode 100755
index 0000000..1621a5e
--- /dev/null
+++ b/ros2_foxy_dev/asus/start_limo_docker.sh
@@ -0,0 +1,33 @@
+#!/bin/bash
+set -e
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+
+xhost +local:docker
+
+if docker container inspect limo_sim >/dev/null 2>&1; then
+  CONTAINER_IMAGE="$(docker container inspect limo_sim --format '{{.Image}}')"
+  LATEST_IMAGE="$(docker image inspect limo_sim:latest --format '{{.Id}}')"
+
+  if [ "${CONTAINER_IMAGE}" != "${LATEST_IMAGE}" ]; then
+    echo "The limo_sim image changed; recreating the container..."
+    docker rm -f limo_sim
+  fi
+fi
+
+if docker container inspect limo_sim >/dev/null 2>&1; then
+  docker start -ai limo_sim
+else
+  docker run -it \
+    --name limo_sim \
+    --gpus all \
+    --network host \
+    --ipc host \
+    -e DISPLAY="${DISPLAY}" \
+    -e QT_X11_NO_MITSHM=1 \
+    -e NVIDIA_VISIBLE_DEVICES=all \
+    -e NVIDIA_DRIVER_CAPABILITIES=all \
+    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
+    -v "${SCRIPT_DIR}/../workspace:/workspace" \
+    limo_sim bash
+fi
````

### [ros2_foxy_dev/lab/Dockerfile](ros2_foxy_dev/lab/Dockerfile)

**Aggiunto: righe 1–103.**

**Origine:** [`d43c9de`](https://github.com/chullius17/Limo/commit/d43c9ded93f70bee238d3ddc69fbdb7205282a73), 18 settembre.

Foxy viene isolato nel container Ubuntu 20.04; l'utente del container usa UID/GID
dell'host per non creare file del workspace con proprietario diverso.
Questo profilo fornisce dipendenze per sviluppo, simulazione e sensori.

````diff
diff --git a/ros2_foxy_dev/lab/Dockerfile b/ros2_foxy_dev/lab/Dockerfile
new file mode 100644
index 0000000..78535c8
--- /dev/null
+++ b/ros2_foxy_dev/lab/Dockerfile
@@ -0,0 +1,103 @@
+FROM ubuntu:20.04
+
+ARG USERNAME=ros
+ARG USER_UID=1000
+ARG USER_GID=1000
+
+ENV DEBIAN_FRONTEND=noninteractive \
+    LANG=en_US.UTF-8 \
+    LC_ALL=en_US.UTF-8 \
+    ROS_DISTRO=foxy \
+    NVIDIA_VISIBLE_DEVICES=all \
+    NVIDIA_DRIVER_CAPABILITIES=all
+
+SHELL ["/bin/bash", "-o", "pipefail", "-c"]
+
+# Base development tools and libraries used by the LIMO workspace.
+RUN apt-get update && apt-get install -y --no-install-recommends \
+    bash-completion \
+    build-essential \
+    ca-certificates \
+    cmake \
+    curl \
+    git \
+    gnupg2 \
+    less \
+    libboost-program-options-dev \
+    libgl1-mesa-glx \
+    libglu1-mesa \
+    libturbojpeg0-dev \
+    libudev-dev \
+    libusb-1.0-0-dev \
+    locales \
+    lsb-release \
+    mesa-utils \
+    nano \
+    python3-pip \
+    python3-setuptools \
+    software-properties-common \
+    sudo \
+    tree \
+    udev \
+    vim \
+    wget \
+    && locale-gen en_US en_US.UTF-8 \
+    && rm -rf /var/lib/apt/lists/*
+
+# ROS 2 Foxy targets Ubuntu 20.04 (Focal), independently of the 24.04 host.
+RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
+      -o /usr/share/keyrings/ros-archive-keyring.gpg \
+    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
+      > /etc/apt/sources.list.d/ros2.list \
+    && apt-get update \
+    && apt-get install -y --no-install-recommends \
+      python3-argcomplete \
+      python3-colcon-common-extensions \
+      python3-matplotlib \
+      python3-numpy \
+      python3-opencv \
+      python3-pyqt5 \
+      python3-pytest \
+      python3-rosdep \
+      python3-scipy \
+      python3-vcstool \
+      python3-yaml \
+      ros-foxy-desktop \
+      ros-foxy-gazebo-ros-pkgs \
+      ros-foxy-joint-state-publisher-gui \
+      ros-foxy-nav2-bringup \
+      ros-foxy-robot-localization \
+      ros-foxy-rqt-robot-steering \
+      ros-foxy-slam-toolbox \
+      ros-foxy-teleop-twist-keyboard \
+      ros-foxy-twist-mux \
+      ros-foxy-xacro \
+    && rm -rf /var/lib/apt/lists/*
+
+# PyTurboJPEG is declared by cv_package. ONNX Runtime is retained for the
+# existing perception pipeline and pinned to a Python 3.8-compatible release.
+RUN python3 -m pip install --no-cache-dir \
+      PyTurboJPEG==1.7.7 \
+      onnxruntime==1.10.0
+
+RUN rosdep init 2>/dev/null || true \
+    && rosdep update --rosdistro foxy
+
+# Match the host UID/GID so files created in the mounted workspace stay owned
+# by the developer instead of root.
+RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
+    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
+    && usermod -aG dialout,video,sudo "${USERNAME}" \
+    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
+    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
+    && echo "source /opt/ros/foxy/setup.bash" >> "/home/${USERNAME}/.bashrc" \
+    && echo 'test ! -f /workspace/install/setup.bash || source /workspace/install/setup.bash' >> "/home/${USERNAME}/.bashrc"
+
+COPY ros-entrypoint.sh /usr/local/bin/ros-entrypoint
+RUN chmod 0755 /usr/local/bin/ros-entrypoint
+
+USER ${USERNAME}
+WORKDIR /workspace
+
+ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
+CMD ["bash"]
````

### [ros2_foxy_dev/lab/compose.yaml](ros2_foxy_dev/lab/compose.yaml)

**Aggiunto: righe 1–29.**

````diff
diff --git a/ros2_foxy_dev/lab/compose.yaml b/ros2_foxy_dev/lab/compose.yaml
new file mode 100644
index 0000000..87befa4
--- /dev/null
+++ b/ros2_foxy_dev/lab/compose.yaml
@@ -0,0 +1,29 @@
+services:
+  foxy:
+    image: limo-foxy-lab:latest
+    container_name: limo-foxy-lab
+    build:
+      context: .
+      args:
+        USER_UID: ${HOST_UID:-1000}
+        USER_GID: ${HOST_GID:-1000}
+    network_mode: host
+    ipc: host
+    privileged: true
+    gpus: all
+    stdin_open: true
+    tty: true
+    environment:
+      DISPLAY: ${DISPLAY:-:0}
+      QT_X11_NO_MITSHM: "1"
+      NVIDIA_VISIBLE_DEVICES: all
+      NVIDIA_DRIVER_CAPABILITIES: all
+      ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
+    volumes:
+      - ../../workspace:/workspace
+      - /tmp/.X11-unix:/tmp/.X11-unix:rw
+      - /dev:/dev
+      - /run/udev:/run/udev:ro
+    working_dir: /workspace
+    command: ["sleep", "infinity"]
+    restart: unless-stopped
````

### [ros2_foxy_dev/lab/dev.sh](ros2_foxy_dev/lab/dev.sh)

**Aggiunto: righe 1–52.**

````diff
diff --git a/ros2_foxy_dev/lab/dev.sh b/ros2_foxy_dev/lab/dev.sh
new file mode 100755
index 0000000..f459334
--- /dev/null
+++ b/ros2_foxy_dev/lab/dev.sh
@@ -0,0 +1,52 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+cd "${SCRIPT_DIR}"
+
+export HOST_UID="${HOST_UID:-$(id -u)}"
+export HOST_GID="${HOST_GID:-$(id -g)}"
+export DISPLAY="${DISPLAY:-:0}"
+
+if ! docker info >/dev/null 2>&1; then
+  echo "Docker non accessibile. Eseguire ./setup-host.sh e poi riconnettersi." >&2
+  exit 1
+fi
+
+allow_x11() {
+  if command -v xhost >/dev/null 2>&1; then
+    xhost +local:docker >/dev/null 2>&1 || \
+      echo "Avviso: X11 non autorizzato; RViz/Gazebo potrebbero non aprirsi." >&2
+  fi
+}
+
+case "${1:-shell}" in
+  build)
+    docker compose build
+    ;;
+  up)
+    allow_x11
+    docker compose up -d --build
+    ;;
+  shell)
+    allow_x11
+    docker compose up -d
+    docker compose exec foxy bash
+    ;;
+  build-workspace)
+    docker compose up -d
+    docker compose exec foxy bash -lc \
+      'sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy && colcon build --symlink-install'
+    ;;
+  gpu)
+    docker compose up -d
+    docker compose exec -T foxy nvidia-smi
+    ;;
+  down)
+    docker compose down
+    ;;
+  *)
+    echo "Uso: $0 {build|up|shell|build-workspace|gpu|down}" >&2
+    exit 2
+    ;;
+esac
````

### [ros2_foxy_dev/lab/ros-entrypoint.sh](ros2_foxy_dev/lab/ros-entrypoint.sh)

**Aggiunto: righe 1–10.**

````diff
diff --git a/ros2_foxy_dev/lab/ros-entrypoint.sh b/ros2_foxy_dev/lab/ros-entrypoint.sh
new file mode 100755
index 0000000..ee17385
--- /dev/null
+++ b/ros2_foxy_dev/lab/ros-entrypoint.sh
@@ -0,0 +1,10 @@
+#!/usr/bin/env bash
+set -e
+
+source /opt/ros/foxy/setup.bash
+
+if [[ -f /workspace/install/setup.bash ]]; then
+  source /workspace/install/setup.bash
+fi
+
+exec "$@"
````

### [ros2_foxy_dev/lab/setup-host.sh](ros2_foxy_dev/lab/setup-host.sh)

**Aggiunto: righe 1–60.**

````diff
diff --git a/ros2_foxy_dev/lab/setup-host.sh b/ros2_foxy_dev/lab/setup-host.sh
new file mode 100755
index 0000000..e243770
--- /dev/null
+++ b/ros2_foxy_dev/lab/setup-host.sh
@@ -0,0 +1,60 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+if [[ "$(. /etc/os-release; printf '%s' "${ID}")" != "ubuntu" ]]; then
+  echo "Questo script supporta solo Ubuntu." >&2
+  exit 1
+fi
+
+if ! command -v nvidia-smi >/dev/null 2>&1; then
+  echo "Driver NVIDIA non rilevato: installarlo e riavviare prima di continuare." >&2
+  exit 1
+fi
+
+# Repair permissions left by an interrupted/older toolkit setup before apt
+# reads an already configured NVIDIA repository.
+if [[ -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg ]]; then
+  sudo chmod a+r /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
+fi
+
+sudo apt-get update
+sudo apt-get install -y ca-certificates curl gnupg
+
+# Docker Engine official apt repository.
+sudo install -m 0755 -d /etc/apt/keyrings
+curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
+  | sudo gpg --dearmor --yes -o /etc/apt/keyrings/docker.gpg
+sudo chmod a+r /etc/apt/keyrings/docker.gpg
+. /etc/os-release
+printf '%s\n' \
+  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${VERSION_CODENAME} stable" \
+  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
+
+sudo apt-get update
+sudo apt-get install -y \
+  docker-ce \
+  docker-ce-cli \
+  containerd.io \
+  docker-buildx-plugin \
+  docker-compose-plugin
+sudo systemctl enable --now docker
+sudo usermod -aG docker "${USER}"
+
+# NVIDIA Container Toolkit official apt repository.
+curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
+  | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
+sudo chmod a+r /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
+curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
+  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
+  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
+
+sudo apt-get update
+sudo apt-get install -y nvidia-container-toolkit
+sudo nvidia-ctk runtime configure --runtime=docker
+sudo systemctl restart docker
+
+sudo docker run --rm hello-world
+sudo docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu24.04 nvidia-smi
+
+echo
+echo "Host pronto. Disconnettersi e riconnettersi per usare Docker senza sudo."
````

### [ros2_foxy_dev/lab_wsl/.env.example](ros2_foxy_dev/lab_wsl/.env.example)

**Aggiunto: righe 1–7.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/.env.example b/ros2_foxy_dev/lab_wsl/.env.example
new file mode 100644
index 0000000..f609766
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/.env.example
@@ -0,0 +1,7 @@
+# Copy to .env; dev.sh reads these two flags to select Compose overlays.
+ENABLE_GPU=0
+ENABLE_HOST_NETWORK=0
+ROS_DOMAIN_ID=0
+ROS_LOCALHOST_ONLY=0
+# Stable GUI fallback for Ubuntu 20.04 on WSLg.
+LIBGL_ALWAYS_SOFTWARE=1
````

### [ros2_foxy_dev/lab_wsl/.gitattributes](ros2_foxy_dev/lab_wsl/.gitattributes)

**Aggiunto: righe 1–4.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/.gitattributes b/ros2_foxy_dev/lab_wsl/.gitattributes
new file mode 100644
index 0000000..e04eae0
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/.gitattributes
@@ -0,0 +1,4 @@
+*.sh text eol=lf
+*.py text eol=lf
+Dockerfile text eol=lf
+*.yaml text eol=lf
````

### [ros2_foxy_dev/lab_wsl/.gitignore](ros2_foxy_dev/lab_wsl/.gitignore)

**Aggiunto: righe 1–2.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/.gitignore b/ros2_foxy_dev/lab_wsl/.gitignore
new file mode 100644
index 0000000..d50a09f
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/.gitignore
@@ -0,0 +1,2 @@
+.env
+__pycache__/
````

### [ros2_foxy_dev/lab_wsl/Dockerfile](ros2_foxy_dev/lab_wsl/Dockerfile)

**Aggiunto: righe 1–81.**

**Origine sul ramo attuale:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683), 4 ottobre.

Il profilo adatta l'ambiente Foxy a Docker Desktop/WSL2 e WSLg. Installa le
dipendenze dai manifest disponibili e compila l'SDK lidar locale. Gli output
di build usano il volume `.lab_wsl`, separato dal normale install del robot.
GPU e rete host sono opzionali; la grafica software è la scelta predefinita.

````diff
diff --git a/ros2_foxy_dev/lab_wsl/Dockerfile b/ros2_foxy_dev/lab_wsl/Dockerfile
new file mode 100644
index 0000000..fa6f1a8
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/Dockerfile
@@ -0,0 +1,81 @@
+FROM ubuntu:20.04
+
+ARG USERNAME=ros
+ARG USER_UID=1000
+ARG USER_GID=1000
+
+ENV DEBIAN_FRONTEND=noninteractive \
+    LANG=en_US.UTF-8 \
+    LC_ALL=en_US.UTF-8 \
+    ROS_DISTRO=foxy
+
+SHELL ["/bin/bash", "-o", "pipefail", "-c"]
+
+# Based on ../lab, with native dependencies found in workspace/src as well.
+RUN apt-get update && apt-get install -y --no-install-recommends \
+    bash-completion build-essential ca-certificates cmake curl git gnupg2 \
+    less locales lsb-release nano software-properties-common sudo tree vim wget \
+    pkg-config python3-dev python3-pip python3-setuptools swig \
+    libboost-program-options-dev libeigen3-dev libgflags-dev libgoogle-glog-dev \
+    libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa libopencv-dev \
+    libturbojpeg0-dev libudev-dev libusb-1.0-0-dev libuvc-dev \
+    mesa-utils udev usbutils \
+    && locale-gen en_US en_US.UTF-8 \
+    && rm -rf /var/lib/apt/lists/*
+
+RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
+      -o /usr/share/keyrings/ros-archive-keyring.gpg \
+    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu focal main" \
+      > /etc/apt/sources.list.d/ros2.list \
+    && apt-get update && apt-get install -y --no-install-recommends \
+      python3-argcomplete python3-colcon-common-extensions python3-matplotlib \
+      python3-numpy python3-opencv python3-pyqt5 python3-pytest python3-rosdep \
+      python3-scipy python3-vcstool python3-yaml \
+      ros-foxy-desktop ros-foxy-gazebo-ros-pkgs ros-foxy-cartographer-ros \
+      ros-foxy-joint-state-publisher-gui ros-foxy-nav2-bringup \
+      ros-foxy-robot-localization ros-foxy-rqt-robot-steering ros-foxy-slam-toolbox \
+      ros-foxy-teleop-twist-keyboard ros-foxy-twist-mux ros-foxy-xacro \
+      ros-foxy-camera-info-manager ros-foxy-cv-bridge ros-foxy-image-geometry \
+      ros-foxy-image-publisher ros-foxy-image-transport ros-foxy-tf2-eigen \
+      ros-foxy-tf2-sensor-msgs \
+    && rm -rf /var/lib/apt/lists/*
+
+COPY ros2_foxy_dev/lab_wsl/requirements.txt /tmp/requirements.txt
+RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt \
+    && python3 -m pip check
+
+# Dockerfile.dockerignore copies only manifests and the local YDLidar SDK.
+# COLCON_IGNORE is deliberately excluded here: install dependencies for the
+# physical drivers too, without enabling them in the mounted workspace.
+COPY workspace/src/ /tmp/workspace-src/
+RUN rosdep init \
+    && rosdep update --rosdistro foxy --include-eol-distros \
+    && apt-get update \
+    && rosdep install --from-paths /tmp/workspace-src --ignore-src -y --rosdistro foxy \
+    && cmake -S /tmp/workspace-src/YDLidar-SDK -B /tmp/ydlidar-build \
+      -DBUILD_SHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
+      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
+    && cmake --build /tmp/ydlidar-build --parallel 2 \
+    && cmake --install /tmp/ydlidar-build \
+    && ldconfig \
+    && rm -rf /tmp/workspace-src /tmp/ydlidar-build /var/lib/apt/lists/*
+
+RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
+    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
+    && usermod -aG dialout,video,sudo "${USERNAME}" \
+    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
+    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
+    && echo 'source /opt/ros/foxy/setup.bash' >> "/home/${USERNAME}/.bashrc" \
+    && echo 'test ! -f /workspace/.lab_wsl/install/setup.bash || source /workspace/.lab_wsl/install/setup.bash' \
+      >> "/home/${USERNAME}/.bashrc"
+
+COPY ros2_foxy_dev/lab_wsl/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
+COPY ros2_foxy_dev/lab_wsl/workspace-deps.sh /usr/local/bin/workspace-deps
+COPY ros2_foxy_dev/lab_wsl/collect-manifests.py /usr/local/bin/collect-manifests
+RUN sed -i 's/\r$//' /usr/local/bin/ros-entrypoint /usr/local/bin/workspace-deps /usr/local/bin/collect-manifests \
+    && chmod 0755 /usr/local/bin/ros-entrypoint /usr/local/bin/workspace-deps /usr/local/bin/collect-manifests
+
+USER ${USERNAME}
+WORKDIR /workspace
+ENTRYPOINT ["/usr/local/bin/ros-entrypoint"]
+CMD ["bash"]
````

### [ros2_foxy_dev/lab_wsl/Dockerfile.dockerignore](ros2_foxy_dev/lab_wsl/Dockerfile.dockerignore)

**Aggiunto: righe 1–20.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/Dockerfile.dockerignore b/ros2_foxy_dev/lab_wsl/Dockerfile.dockerignore
new file mode 100644
index 0000000..a6f9a80
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/Dockerfile.dockerignore
@@ -0,0 +1,20 @@
+# This build uses the repository root as its context.
+**
+!ros2_foxy_dev/
+!ros2_foxy_dev/lab_wsl/
+!ros2_foxy_dev/lab_wsl/Dockerfile
+!ros2_foxy_dev/lab_wsl/requirements.txt
+!ros2_foxy_dev/lab_wsl/ros-entrypoint.sh
+!ros2_foxy_dev/lab_wsl/workspace-deps.sh
+!ros2_foxy_dev/lab_wsl/collect-manifests.py
+!workspace/
+!workspace/src/
+!workspace/src/**/
+!workspace/src/**/package.xml
+!workspace/src/YDLidar-SDK/**
+**/.git
+**/__pycache__
+**/build
+**/install
+**/log
+**/COLCON_IGNORE
````

### [ros2_foxy_dev/lab_wsl/collect-manifests.py](ros2_foxy_dev/lab_wsl/collect-manifests.py)

**Aggiunto: righe 1–40.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/collect-manifests.py b/ros2_foxy_dev/lab_wsl/collect-manifests.py
new file mode 100755
index 0000000..cc63791
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/collect-manifests.py
@@ -0,0 +1,40 @@
+#!/usr/bin/env python3
+"""Collect all source manifests, including packages with COLCON_IGNORE."""
+import argparse
+import os
+from pathlib import Path
+import shutil
+import xml.etree.ElementTree as ET
+
+
+def main():
+    parser = argparse.ArgumentParser(description=__doc__)
+    parser.add_argument("source", type=Path)
+    parser.add_argument("destination", type=Path)
+    args = parser.parse_args()
+    if not args.source.is_dir():
+        parser.error("Source directory does not exist: {}".format(args.source))
+    packages = {}
+    for directory, subdirs, files in os.walk(str(args.source)):
+        subdirs[:] = sorted(d for d in subdirs if d not in
+                            {".git", "__pycache__", "build", "install", "log"})
+        if "package.xml" not in files:
+            continue
+        manifest = Path(directory) / "package.xml"
+        name = ET.parse(str(manifest)).getroot().findtext("name")
+        if not name or "/" in name or name in {".", ".."}:
+            parser.error("Invalid package name in {}".format(manifest))
+        if name in packages:
+            parser.error("Duplicate package {}: {}, {}".format(name, packages[name], manifest))
+        packages[name] = manifest
+    if not packages:
+        parser.error("No package.xml found in {}".format(args.source))
+    for name, manifest in sorted(packages.items()):
+        target = args.destination / name
+        target.mkdir(parents=True, exist_ok=True)
+        shutil.copyfile(str(manifest), str(target / "package.xml"))
+    print("Collected {} package manifests".format(len(packages)))
+
+
+if __name__ == "__main__":
+    main()
````

### [ros2_foxy_dev/lab_wsl/compose.gpu.yaml](ros2_foxy_dev/lab_wsl/compose.gpu.yaml)

**Aggiunto: righe 1–13.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/compose.gpu.yaml b/ros2_foxy_dev/lab_wsl/compose.gpu.yaml
new file mode 100644
index 0000000..2104f1e
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/compose.gpu.yaml
@@ -0,0 +1,13 @@
+# Optional NVIDIA compute support; software OpenGL remains the WSLg fallback.
+services:
+  foxy:
+    deploy:
+      resources:
+        reservations:
+          devices:
+            - driver: nvidia
+              count: all
+              capabilities: [gpu]
+    environment:
+      NVIDIA_VISIBLE_DEVICES: all
+      NVIDIA_DRIVER_CAPABILITIES: compute,utility
````

### [ros2_foxy_dev/lab_wsl/compose.host.yaml](ros2_foxy_dev/lab_wsl/compose.host.yaml)

**Aggiunto: righe 1–4.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/compose.host.yaml b/ros2_foxy_dev/lab_wsl/compose.host.yaml
new file mode 100644
index 0000000..8cb77ed
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/compose.host.yaml
@@ -0,0 +1,4 @@
+# Requires Docker Desktop >= 4.34 and "Enable host networking" in Settings.
+services:
+  foxy:
+    network_mode: host
````

### [ros2_foxy_dev/lab_wsl/compose.yaml](ros2_foxy_dev/lab_wsl/compose.yaml)

**Aggiunto: righe 1–52.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/compose.yaml b/ros2_foxy_dev/lab_wsl/compose.yaml
new file mode 100644
index 0000000..89ac5cd
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/compose.yaml
@@ -0,0 +1,52 @@
+name: limo-foxy-lab-wsl
+
+services:
+  foxy:
+    image: limo-foxy-lab-wsl:latest
+    platform: linux/amd64
+    build:
+      context: ../..
+      dockerfile: ros2_foxy_dev/lab_wsl/Dockerfile
+      args:
+        USER_UID: ${HOST_UID:-1000}
+        USER_GID: ${HOST_GID:-1000}
+    stdin_open: true
+    tty: true
+    init: true
+    shm_size: 1gb
+    environment:
+      ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
+      ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY:-0}
+      DISPLAY: ${DISPLAY:-:0}
+      WAYLAND_DISPLAY: ${WAYLAND_DISPLAY:-wayland-0}
+      XDG_RUNTIME_DIR: /mnt/wslg/runtime-dir
+      PULSE_SERVER: ${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}
+      QT_X11_NO_MITSHM: "1"
+      LIBGL_ALWAYS_SOFTWARE: ${LIBGL_ALWAYS_SOFTWARE:-1}
+    volumes:
+      - type: bind
+        source: ../../workspace
+        target: /workspace
+        bind:
+          create_host_path: false
+      - type: volume
+        source: wsl_build
+        target: /workspace/.lab_wsl
+      - type: bind
+        source: /mnt/wslg
+        target: /mnt/wslg
+        read_only: true
+        bind:
+          create_host_path: false
+      - type: bind
+        source: /mnt/wslg/.X11-unix
+        target: /tmp/.X11-unix
+        read_only: true
+        bind:
+          create_host_path: false
+    working_dir: /workspace
+    command: ["sleep", "infinity"]
+    restart: unless-stopped
+
+volumes:
+  wsl_build:
````

### [ros2_foxy_dev/lab_wsl/dev.sh](ros2_foxy_dev/lab_wsl/dev.sh)

**Aggiunto: righe 1–83.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/dev.sh b/ros2_foxy_dev/lab_wsl/dev.sh
new file mode 100755
index 0000000..ed8d942
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/dev.sh
@@ -0,0 +1,83 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+cd "$SCRIPT_DIR"
+
+# Load only explicit profile options; Compose itself reads .env for substitutions.
+if [[ -f .env ]]; then
+  while IFS= read -r line || [[ -n "$line" ]]; do
+    line="${line%$'\r'}"
+    case "$line" in
+      ENABLE_GPU=0|ENABLE_GPU=1|ENABLE_HOST_NETWORK=0|ENABLE_HOST_NETWORK=1)
+        export "$line"
+        ;;
+    esac
+  done < .env
+fi
+export HOST_UID="${HOST_UID:-$(id -u)}"
+export HOST_GID="${HOST_GID:-$(id -g)}"
+compose=(docker compose -f compose.yaml)
+[[ "${ENABLE_GPU:-0}" != 1 ]] || compose+=(-f compose.gpu.yaml)
+[[ "${ENABLE_HOST_NETWORK:-0}" != 1 ]] || compose+=(-f compose.host.yaml)
+
+usage() {
+  echo "Uso: $0 {build|up|shell|build-workspace|deps|check|gpu|config|down}" >&2
+}
+
+action="${1:-shell}"
+case "$action" in
+  build|up|shell|build-workspace|deps|check|gpu|config|down) ;;
+  *) usage; exit 2 ;;
+esac
+
+if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
+  echo "Docker Desktop/Compose non disponibile. Abilitare WSL Integration per questa distro." >&2
+  exit 1
+fi
+if [[ "$action" == config ]]; then
+  "${compose[@]}" config
+  exit
+fi
+if ! docker info >/dev/null 2>&1; then
+  echo "Docker non accessibile: avviare Docker Desktop e abilitare WSL Integration." >&2
+  exit 1
+fi
+if [[ "$action" != build && "$action" != down ]]; then
+  if [[ ! -d /mnt/wslg/.X11-unix ]]; then
+    echo "WSLg non disponibile. Eseguire wsl --update su Windows e riavviare WSL." >&2
+    exit 1
+  fi
+fi
+
+case "$action" in
+  build) "${compose[@]}" build ;;
+  up) "${compose[@]}" up -d --build ;;
+  shell)
+    "${compose[@]}" up -d
+    "${compose[@]}" exec foxy bash
+    ;;
+  build-workspace)
+    "${compose[@]}" up -d
+    "${compose[@]}" exec foxy bash -c \
+      'source /opt/ros/foxy/setup.bash && colcon --log-base /workspace/.lab_wsl/log build --symlink-install --build-base /workspace/.lab_wsl/build --install-base /workspace/.lab_wsl/install --parallel-workers 2'
+    ;;
+  deps)
+    "${compose[@]}" up -d
+    "${compose[@]}" exec foxy workspace-deps
+    ;;
+  check)
+    "${compose[@]}" up -d
+    "${compose[@]}" exec -T foxy /usr/local/bin/ros-entrypoint bash -c \
+      'python3 -c "import rclpy, cv2, numpy, scipy, yaml, matplotlib, onnxruntime, turbojpeg; from PyQt5 import QtCore; from cv_bridge import CvBridge; from turbojpeg import TurboJPEG; TurboJPEG(); print(\"Import e TurboJPEG OK\")" && python3 -m pip check && ros2 pkg prefix gazebo_ros && ros2 pkg prefix nav2_bringup && ros2 pkg prefix cartographer_ros && pkg-config --modversion libuvc libglog && test -f /usr/local/lib/libydlidar_sdk.so && colcon list --base-paths src'
+    ;;
+  gpu)
+    if [[ "${ENABLE_GPU:-0}" != 1 ]]; then
+      echo "Impostare ENABLE_GPU=1 in .env prima di usare gpu (richiede NVIDIA)." >&2
+      exit 1
+    fi
+    "${compose[@]}" up -d
+    "${compose[@]}" exec -T foxy nvidia-smi
+    ;;
+  down) "${compose[@]}" down ;;
+esac
````

### [ros2_foxy_dev/lab_wsl/requirements.txt](ros2_foxy_dev/lab_wsl/requirements.txt)

**Aggiunto: righe 1–7.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/requirements.txt b/ros2_foxy_dev/lab_wsl/requirements.txt
new file mode 100644
index 0000000..4c11bf8
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/requirements.txt
@@ -0,0 +1,7 @@
+# Python 3.8 / ROS Foxy; retained from the lab perception environment.
+# NumPy < 1.24 keeps aliases used by Ubuntu Focal's scientific packages.
+numpy==1.23.5
+protobuf==3.20.3
+flatbuffers==2.0.7
+PyTurboJPEG==1.7.7
+onnxruntime==1.10.0
````

### [ros2_foxy_dev/lab_wsl/ros-entrypoint.sh](ros2_foxy_dev/lab_wsl/ros-entrypoint.sh)

**Aggiunto: righe 1–12.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/ros-entrypoint.sh b/ros2_foxy_dev/lab_wsl/ros-entrypoint.sh
new file mode 100755
index 0000000..4405859
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/ros-entrypoint.sh
@@ -0,0 +1,12 @@
+#!/usr/bin/env bash
+set -e
+
+# Only this profile's named volume; never reuse the lab/host build or install.
+if [[ -d /workspace/.lab_wsl ]]; then
+  sudo chown "$(id -u):$(id -g)" /workspace/.lab_wsl
+fi
+source /opt/ros/foxy/setup.bash
+if [[ -f /workspace/.lab_wsl/install/setup.bash ]]; then
+  source /workspace/.lab_wsl/install/setup.bash
+fi
+exec "$@"
````

### [ros2_foxy_dev/lab_wsl/setup-host.sh](ros2_foxy_dev/lab_wsl/setup-host.sh)

**Aggiunto: righe 1–35.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/setup-host.sh b/ros2_foxy_dev/lab_wsl/setup-host.sh
new file mode 100755
index 0000000..f93b0f9
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/setup-host.sh
@@ -0,0 +1,35 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+if ! grep -qi microsoft /proc/sys/kernel/osrelease; then
+  echo "Eseguire questo script dalla distro Ubuntu in WSL2." >&2
+  exit 1
+fi
+if [[ "$(id -u)" == 0 ]]; then
+  echo "Usare l'utente normale della distro WSL, per mantenere corretti UID/GID." >&2
+  exit 1
+fi
+if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
+  echo "Abilitare Ubuntu in Docker Desktop > Settings > Resources > WSL Integration." >&2
+  exit 1
+fi
+if ! docker info >/dev/null 2>&1; then
+  echo "Avviare Docker Desktop e verificare WSL Integration." >&2
+  exit 1
+fi
+if [[ "$(docker info --format '{{.OSType}}')" != linux ]]; then
+  echo "Selezionare Linux containers in Docker Desktop." >&2
+  exit 1
+fi
+if [[ ! -d /mnt/wslg/.X11-unix ]]; then
+  echo "WSLg non disponibile: eseguire wsl --update su Windows, poi riavviare WSL." >&2
+  exit 1
+fi
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+if [[ ! -d "$SCRIPT_DIR/../../workspace/src" ]]; then
+  echo "Manca workspace/src: copiare l'intero progetto in WSL." >&2
+  exit 1
+fi
+docker version
+docker compose version
+echo "Host pronto. Il motore Docker e gestito da Docker Desktop; proseguire con bash dev.sh build."
````

### [ros2_foxy_dev/lab_wsl/setup-windows.ps1](ros2_foxy_dev/lab_wsl/setup-windows.ps1)

**Aggiunto: righe 1–61.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/setup-windows.ps1 b/ros2_foxy_dev/lab_wsl/setup-windows.ps1
new file mode 100644
index 0000000..d8988fb
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/setup-windows.ps1
@@ -0,0 +1,61 @@
+#Requires -Version 5.1
+#Requires -RunAsAdministrator
+[CmdletBinding()]
+param()
+
+$ErrorActionPreference = 'Stop'
+if (-not [Environment]::Is64BitOperatingSystem -or $env:PROCESSOR_ARCHITECTURE -eq 'ARM64') {
+    throw 'Questo profilo richiede Windows x86_64 (immagine linux/amd64).'
+}
+if (-not (Get-Command winget.exe -ErrorAction SilentlyContinue)) {
+    throw 'Installare/aggiornare App Installer (winget) dal Microsoft Store, poi rieseguire.'
+}
+if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
+    throw 'wsl.exe non disponibile: aggiornare Windows e abilitare Windows Subsystem for Linux.'
+}
+
+function Invoke-Checked {
+    param([string]$Program, [string[]]$Arguments)
+    & $Program @Arguments
+    if ($LASTEXITCODE -eq 3010 -or $LASTEXITCODE -eq 1641) {
+        Write-Host 'Riavvio richiesto. Riavviare Windows, poi rieseguire questo script.'
+        exit 0
+    }
+    if ($LASTEXITCODE -ne 0) {
+        throw "$Program non riuscito (codice $LASTEXITCODE). Dopo un riavvio, rieseguire lo script."
+    }
+}
+
+# No forced reboot or unattended acceptance of Docker Desktop's agreement.
+Invoke-Checked -Program 'wsl.exe' -Arguments @('--install', '--no-distribution')
+Invoke-Checked -Program 'wsl.exe' -Arguments @('--update')
+Invoke-Checked -Program 'wsl.exe' -Arguments @('--set-default-version', '2')
+
+$distroOutput = & wsl.exe --list --quiet
+if ($LASTEXITCODE -ne 0) {
+    throw 'Impossibile elencare le distro WSL. Riavviare Windows, poi rieseguire.'
+}
+$distros = @($distroOutput | ForEach-Object { ($_ -replace "`0", '').Trim() })
+if ($distros -notcontains 'Ubuntu-20.04') {
+    Invoke-Checked -Program 'wsl.exe' -Arguments @('--install', '--distribution', 'Ubuntu-20.04', '--no-launch')
+}
+Invoke-Checked -Program 'wsl.exe' -Arguments @('--set-version', 'Ubuntu-20.04', '2')
+
+# Both the installation and the subsequent Desktop settings are visible to the user.
+$dockerPackages = & winget.exe list --exact --id Docker.DockerDesktop --accept-source-agreements
+if ($LASTEXITCODE -ne 0) {
+    Invoke-Checked -Program 'winget.exe' -Arguments @(
+        'install', '--exact', '--id', 'Docker.DockerDesktop', '--source', 'winget',
+        '--accept-source-agreements'
+    )
+} else {
+    Write-Host 'Docker Desktop gia installato:'
+    $dockerPackages | Write-Host
+}
+
+Write-Host ''
+Write-Host 'Aprire Ubuntu-20.04 con: wsl -d Ubuntu-20.04 e creare utente/password al primo avvio.'
+Write-Host 'Aprire Docker Desktop dal menu Start e completare il primo avvio.'
+Write-Host 'Settings > General: Use WSL 2 based engine (se visibile).'
+Write-Host 'Settings > Resources > WSL Integration: abilitare Ubuntu-20.04, poi Apply.'
+Write-Host 'Dalla shell Ubuntu seguire README.md: bash dev.sh build, up, build-workspace, shell.'
````

### [ros2_foxy_dev/lab_wsl/workspace-deps.sh](ros2_foxy_dev/lab_wsl/workspace-deps.sh)

**Aggiunto: righe 1–9.**

````diff
diff --git a/ros2_foxy_dev/lab_wsl/workspace-deps.sh b/ros2_foxy_dev/lab_wsl/workspace-deps.sh
new file mode 100755
index 0000000..10b155f
--- /dev/null
+++ b/ros2_foxy_dev/lab_wsl/workspace-deps.sh
@@ -0,0 +1,9 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+manifest_dir="$(mktemp -d)"
+trap 'rm -rf "$manifest_dir"' EXIT
+collect-manifests /workspace/src "$manifest_dir"
+sudo apt-get update
+sudo rosdep update --rosdistro foxy --include-eol-distros
+sudo rosdep install --from-paths "$manifest_dir" --ignore-src -y --rosdistro foxy
````

### [ros2_foxy_dev/limo/.gitattributes](ros2_foxy_dev/limo/.gitattributes)

**Aggiunto: righe 1–6.**

````diff
diff --git a/ros2_foxy_dev/limo/.gitattributes b/ros2_foxy_dev/limo/.gitattributes
new file mode 100644
index 0000000..cc7520b
--- /dev/null
+++ b/ros2_foxy_dev/limo/.gitattributes
@@ -0,0 +1,6 @@
+*.sh text eol=lf
+*.bash text eol=lf
+*.py text eol=lf
+*.txt text eol=lf
+*.rules text eol=lf
+*.pref text eol=lf
````

### [ros2_foxy_dev/limo/.gitignore](ros2_foxy_dev/limo/.gitignore)

**Aggiunto: righe 1–1.**

````diff
diff --git a/ros2_foxy_dev/limo/.gitignore b/ros2_foxy_dev/limo/.gitignore
new file mode 100644
index 0000000..c18dd8d
--- /dev/null
+++ b/ros2_foxy_dev/limo/.gitignore
@@ -0,0 +1 @@
+__pycache__/
````

### [ros2_foxy_dev/limo/99-limo-hardware.rules](ros2_foxy_dev/limo/99-limo-hardware.rules)

**Aggiunto: righe 1–8.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683).

L'accesso ai dispositivi viene preparato tramite gruppi `dialout`/`video`
e regole udev per UART chassis, adattatori YDLidar e vendor USB Orbbec.
Il nome `/dev/ydlidar` diventa stabile per gli ID previsti; l'adesione ai
gruppi richiede una nuova sessione. Sono requisiti per usare i driver senza
eseguirli come root, non calibrazioni dei sensori.

````diff
diff --git a/ros2_foxy_dev/limo/99-limo-hardware.rules b/ros2_foxy_dev/limo/99-limo-hardware.rules
new file mode 100644
index 0000000..3937c06
--- /dev/null
+++ b/ros2_foxy_dev/limo/99-limo-hardware.rules
@@ -0,0 +1,8 @@
+# Onboard Jetson chassis UART.
+SUBSYSTEM=="tty", KERNEL=="ttyTHS1", GROUP="dialout", MODE="0660"
+# YDLidar adapter IDs from the workspace's existing driver rules.
+SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="ydlidar"
+SUBSYSTEM=="tty", KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", GROUP="dialout", MODE="0660", SYMLINK+="ydlidar"
+SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", GROUP="dialout", MODE="0660", SYMLINK+="ydlidar"
+# Orbbec/Astra/Dabai USB devices, including Dabai U3 product 050e.
+SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", GROUP="video", MODE="0660"
````

### [ros2_foxy_dev/limo/aliases/limo.bash](ros2_foxy_dev/limo/aliases/limo.bash)

**Aggiunto: righe 1–13.**

**Origine:** [`1d296de`](https://github.com/chullius17/Limo/commit/1d296de02cbfd2c2cadda3faafbe4a1004597a56), con percorso dell'install aggiornato in [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a).

`cb_limo` richiama la build robot, `wsp` entra nel workspace e `start` richiama
l'avvio hardware. `setup-shell.sh` inserisce un blocco delimitato in `.bashrc`,
carica Foxy/install/alias e conserva una copia di backup. Rilanciarlo aggiorna
il suo blocco senza moltiplicare le righe di setup. Questa modifica rende
più semplice l'uso quotidiano, senza cambiare il comportamento del driver.

````diff
diff --git a/ros2_foxy_dev/limo/aliases/limo.bash b/ros2_foxy_dev/limo/aliases/limo.bash
new file mode 100644
index 0000000..18881bd
--- /dev/null
+++ b/ros2_foxy_dev/limo/aliases/limo.bash
@@ -0,0 +1,13 @@
+# Source this file from Bash to load the physical robot shortcuts.
+_limo_aliases_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
+
+printf -v _limo_alias_command 'bash %q' "$_limo_aliases_dir/../build-robot.sh"
+alias cb_limo="$_limo_alias_command"
+
+printf -v _limo_alias_command 'cd -- %q' "$_limo_aliases_dir/../../../workspace"
+alias wsp="$_limo_alias_command"
+
+printf -v _limo_alias_command 'bash %q' "$_limo_aliases_dir/../launch-robot.sh"
+alias start="$_limo_alias_command"
+
+unset _limo_aliases_dir _limo_alias_command
````

### [ros2_foxy_dev/limo/apt-packages.txt](ros2_foxy_dev/limo/apt-packages.txt)

**Aggiunto: righe 1–52.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683); `rqt_image_view` aggiunto in [`f86a3e8`](https://github.com/chullius17/Limo/commit/f86a3e80cd29fffd897c930b93883fc4b89e965c).

La lista copre ROS, dipendenze native di chassis/camera/lidar, EKF, strumenti
di navigazione e diagnostica. `rqt_image_view` permette di osservare i flussi
RGB/depth prima di usare una pipeline CV. Il profilo fisico non installa
ONNX Runtime o PyTurboJPEG: i sorgenti applicativi che li usavano sono stati
eliminati da questo ramo e il bringup disponibile non li importa.

````diff
diff --git a/ros2_foxy_dev/limo/apt-packages.txt b/ros2_foxy_dev/limo/apt-packages.txt
new file mode 100644
index 0000000..3472270
--- /dev/null
+++ b/ros2_foxy_dev/limo/apt-packages.txt
@@ -0,0 +1,52 @@
+# Ubuntu 20.04 development and hardware libraries.
+build-essential
+ca-certificates
+cmake
+curl
+git
+gnupg2
+locales
+pkg-config
+python3-dev
+python3-setuptools
+python3-colcon-common-extensions
+python3-rosdep
+python3-argcomplete
+python3-numpy
+python3-opencv
+python3-scipy
+python3-yaml
+libeigen3-dev
+libgflags-dev
+libgoogle-glog-dev
+libopencv-dev
+libusb-1.0-0-dev
+libudev-dev
+libuvc-dev
+udev
+usbutils
+# ROS base, robot launch, navigation and optional visualization. No Gazebo.
+ros-foxy-ros-base
+ros-foxy-launch-xml
+ros-foxy-launch-yaml
+ros-foxy-robot-localization
+ros-foxy-navigation2
+ros-foxy-nav2-bringup
+ros-foxy-slam-toolbox
+ros-foxy-cartographer-ros
+ros-foxy-teleop-twist-keyboard
+ros-foxy-twist-mux
+ros-foxy-rviz2
+ros-foxy-rqt-robot-steering
+ros-foxy-rqt-image-view
+ros-foxy-robot-state-publisher
+ros-foxy-joint-state-publisher
+ros-foxy-joint-state-publisher-gui
+ros-foxy-xacro
+ros-foxy-camera-info-manager
+ros-foxy-cv-bridge
+ros-foxy-image-geometry
+ros-foxy-image-publisher
+ros-foxy-image-transport
+ros-foxy-tf2-eigen
+ros-foxy-tf2-sensor-msgs
````

### [ros2_foxy_dev/limo/build-robot.sh](ros2_foxy_dev/limo/build-robot.sh)

**Aggiunto: righe 1–87.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683); aggiunta di alias/symlink in [`1d296de`](https://github.com/chullius17/Limo/commit/1d296de02cbfd2c2cadda3faafbe4a1004597a56);
passaggio alla build diretta dei sorgenti in [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a).

La prima versione preparava copie dei pacchetti in `workspace/.limo`.
L'ultima compila i sorgenti originali selezionati con `--base-paths`, usando
`workspace/build`, `workspace/install` e `workspace/log`: si modifica una
sola copia del codice e si ricompila quella. Vengono esclusi dalla discovery
i pacchetti simulatori e rimossi i marker COLCON_IGNORE dei driver selezionati.

L'SDK YDLidar viene compilato prima, perché il manifest del driver non dichiara
quel passo di build. È statico e installato privatamente in
`workspace/install/ydlidar_sdk`, senza sostituire l'SDK di sistema.
Vengono azzerati i prefissi di overlay prima di caricare Foxy, evitando di
trascinare una precedente installazione nella nuova build.

Due worker sono il default per limitare il consumo di RAM. `--without-camera`
permette la build anche senza runtime Orbbec e salva la scelta per l'avvio;
`BUILD_TESTING=OFF` significa che questa procedura non esegue i test del progetto.
Alla fine viene configurata la shell del robot.

````diff
diff --git a/ros2_foxy_dev/limo/build-robot.sh b/ros2_foxy_dev/limo/build-robot.sh
new file mode 100644
index 0000000..bd70dda
--- /dev/null
+++ b/ros2_foxy_dev/limo/build-robot.sh
@@ -0,0 +1,87 @@
+#!/usr/bin/env bash
+set -eo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
+SDK_BUILD_DIR="$WORKSPACE_DIR/build/ydlidar_sdk"
+SDK_INSTALL_DIR="$WORKSPACE_DIR/install/ydlidar_sdk"
+with_camera=1
+case "${1:-}" in
+  "") ;;
+  --without-camera) with_camera=0 ;;
+  *) echo "Usage: bash build-robot.sh [--without-camera]" >&2; exit 2 ;;
+esac
+if [[ $# -gt 1 ]]; then
+  echo "Usage: bash build-robot.sh [--without-camera]" >&2
+  exit 2
+fi
+if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
+  echo "ROS 2 Foxy is missing. Run bash install-deps.sh first." >&2
+  exit 1
+fi
+if [[ ! -f "$WORKSPACE_DIR/src/YDLidar-SDK/CMakeLists.txt" ]]; then
+  echo "Missing workspace/src/YDLidar-SDK sources." >&2
+  exit 1
+fi
+if [[ "$with_camera" == 1 ]]; then
+  case "$(dpkg --print-architecture)" in
+    arm64) camera_platform=arm64 ;;
+    amd64) camera_platform=x64 ;;
+    *) echo "Unsupported camera architecture." >&2; exit 1 ;;
+  esac
+  redist="$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera/openni2_redist/$camera_platform"
+  if [[ ! -f "$redist/libOpenNI2.so" ]] || ! compgen -G "$redist/OpenNI2/Drivers/*.so" >/dev/null; then
+    echo "Missing Orbbec OpenNI2 runtime for $camera_platform: $redist" >&2
+    echo "Restore the manufacturer's SDK files, or build with --without-camera." >&2
+    exit 1
+  fi
+fi
+package_paths=(
+  "$WORKSPACE_DIR/src/limo_ros2/limo_msgs"
+  "$WORKSPACE_DIR/src/limo_ros2/limo_base"
+  "$WORKSPACE_DIR/src/limo_ros2/limo_bringup"
+  "$WORKSPACE_DIR/src/limo_ros2/limo_description"
+  "$WORKSPACE_DIR/src/custom_start"
+  "$WORKSPACE_DIR/src/ydlidar_ros2_driver"
+)
+if [[ "$with_camera" == 1 ]]; then
+  package_paths+=(
+    "$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera_msgs"
+    "$WORKSPACE_DIR/src/ros2_astra_camera/astra_camera"
+  )
+fi
+for package_path in "${package_paths[@]}"; do
+  if [[ ! -f "$package_path/package.xml" ]]; then
+    echo "Missing robot package manifest: $package_path/package.xml" >&2
+    exit 1
+  fi
+done
+
+# Build against Foxy, without chaining the previous generated robot workspace
+# or another build of this workspace into the new install/setup.bash.
+unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
+source /opt/ros/foxy/setup.bash
+for package_path in "${package_paths[@]}"; do
+  if [[ -e "$package_path/COLCON_IGNORE" || -L "$package_path/COLCON_IGNORE" ]]; then
+    rm -f "$package_path/COLCON_IGNORE"
+  fi
+done
+
+# Build the C++ SDK before colcon: the driver does not declare this build edge.
+# A private static SDK avoids replacing a system-wide or existing lab SDK.
+cmake -S "$WORKSPACE_DIR/src/YDLidar-SDK" -B "$SDK_BUILD_DIR" \
+  -DCMAKE_INSTALL_PREFIX="$SDK_INSTALL_DIR" \
+  -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF \
+  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON -DSWIG_FOUND=FALSE
+cmake --build "$SDK_BUILD_DIR" --parallel "${BUILD_JOBS:-2}"
+cmake --install "$SDK_BUILD_DIR"
+export CMAKE_PREFIX_PATH="$SDK_INSTALL_DIR${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
+cd "$WORKSPACE_DIR"
+colcon --log-base "$WORKSPACE_DIR/log" build \
+  --symlink-install \
+  --base-paths "${package_paths[@]}" \
+  --build-base "$WORKSPACE_DIR/build" --install-base "$WORKSPACE_DIR/install" \
+  --parallel-workers "${BUILD_JOBS:-2}" --cmake-args -DBUILD_TESTING=OFF
+printf '%s\n' "$with_camera" > "$WORKSPACE_DIR/install/limo-camera-enabled"
+bash "$SCRIPT_DIR/setup-shell.sh"
+echo "Robot workspace built. Start it with bash launch-robot.sh."
````

### [ros2_foxy_dev/limo/install-deps.sh](ros2_foxy_dev/limo/install-deps.sh)

**Aggiunto: righe 1–50.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683), adattato al layout radice in [`8624208`](https://github.com/chullius17/Limo/commit/8624208a118fb29b25153214994a631b64373152).

Prepara Ubuntu 20.04 arm64/amd64 direttamente sul robot, installando pacchetti
apt e dipendenze ROS con rosdep. Usa copie temporanee dei soli manifest
fisici per eliminare le dipendenze simulatori; i sorgenti originali rimangono
quelli usati dalla build. Il pin apt temporaneo impedisce installazioni Gazebo
anche attraverso dipendenze transitive. Installare Nav2/SLAM qui non avvia
una pipeline di navigazione o ripristina i pacchetti applicativi cancellati.

````diff
diff --git a/ros2_foxy_dev/limo/install-deps.sh b/ros2_foxy_dev/limo/install-deps.sh
new file mode 100644
index 0000000..8446d1a
--- /dev/null
+++ b/ros2_foxy_dev/limo/install-deps.sh
@@ -0,0 +1,50 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
+if [[ "$(id -u)" == 0 ]]; then
+  echo "Run as the normal robot user; the script requests sudo when needed." >&2
+  exit 1
+fi
+. /etc/os-release
+if [[ "$ID" != ubuntu || "$VERSION_ID" != 20.04 ]]; then
+  echo "This profile requires Ubuntu 20.04. Detected: $PRETTY_NAME" >&2
+  exit 1
+fi
+case "$(dpkg --print-architecture)" in
+  amd64|arm64) ;;
+  *) echo "Supported architectures: amd64 and arm64." >&2; exit 1 ;;
+esac
+
+temp_dir="$(mktemp -d)"
+pin_path=""
+cleanup() {
+  [[ -z "$pin_path" ]] || sudo rm -f "$pin_path"
+  rm -rf "$temp_dir"
+}
+trap cleanup EXIT
+python3 "$SCRIPT_DIR/prepare-workspace.py" "$WORKSPACE_DIR/src" "$temp_dir/manifests" --manifests-only
+sudo -v
+# Refuse new Gazebo packages, including transitive dependencies. Removed on exit.
+pin_path="$(sudo mktemp /etc/apt/preferences.d/limo-no-gazebo-XXXXXX)"
+sudo install -m 0644 "$SCRIPT_DIR/no-gazebo.pref" "$pin_path"
+sudo apt-get update
+sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2 locales software-properties-common
+sudo add-apt-repository -y universe
+sudo locale-gen en_US.UTF-8
+curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$temp_dir/ros.key"
+sudo install -m 0644 "$temp_dir/ros.key" /usr/share/keyrings/ros-archive-keyring.gpg
+printf '%s\n' \
+  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
+  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
+sudo apt-get update
+mapfile -t packages < <(sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$SCRIPT_DIR/apt-packages.txt")
+sudo apt-get install -y --no-install-recommends "${packages[@]}"
+if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
+  sudo rosdep init
+fi
+rosdep update --rosdistro foxy --include-eol-distros
+rosdep install --from-paths "$temp_dir/manifests" --ignore-src -y --rosdistro foxy
+echo "System dependencies installed. Next: bash setup-devices.sh, then bash build-robot.sh."
+echo "The YDLidar SDK is built by build-robot.sh. Orbbec's OpenNI2 files must be supplied separately."
````

### [ros2_foxy_dev/limo/launch-robot.sh](ros2_foxy_dev/limo/launch-robot.sh)

**Aggiunto: righe 1–17.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683); percorsi aggiornati in [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a).

Carica Foxy e `workspace/install/setup.bash`, verifica che la build robot sia
stata completata e lancia `custom_start/limo_real.launch.py`. Se la build
era senza camera, aggiunge `use_camera:=false`. Gli argomenti successivi
vengono inoltrati al launch e ROS_DOMAIN_ID è configurabile.
L'avvio prepara il grafo hardware/EKF, senza lanciare automaticamente SLAM o Nav2.

````diff
diff --git a/ros2_foxy_dev/limo/launch-robot.sh b/ros2_foxy_dev/limo/launch-robot.sh
new file mode 100644
index 0000000..b27db19
--- /dev/null
+++ b/ros2_foxy_dev/limo/launch-robot.sh
@@ -0,0 +1,17 @@
+#!/usr/bin/env bash
+set -eo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../workspace" && pwd)"
+if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" || ! -f "$WORKSPACE_DIR/install/limo-camera-enabled" ]]; then
+  echo "Build the robot workspace first: bash build-robot.sh [--without-camera]." >&2
+  exit 1
+fi
+source /opt/ros/foxy/setup.bash
+source "$WORKSPACE_DIR/install/setup.bash"
+export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
+camera_args=()
+if [[ "$(cat "$WORKSPACE_DIR/install/limo-camera-enabled")" == 0 ]]; then
+  camera_args+=(use_camera:=false)
+fi
+exec ros2 launch custom_start limo_real.launch.py "${camera_args[@]}" "$@"
````

### [ros2_foxy_dev/limo/no-gazebo.pref](ros2_foxy_dev/limo/no-gazebo.pref)

**Aggiunto: righe 1–4.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683).

Il manifest `custom_start` combina dipendenze reali e simulate. Oltre alla
preparazione dei manifest, questo pin a priorità −1 impedisce che apt aggiunga
pacchetti Gazebo durante l'installazione del profilo robot. Il pin viene
rimosso all'uscita dell'installer; Gazebo già installato non viene cancellato.

````diff
diff --git a/ros2_foxy_dev/limo/no-gazebo.pref b/ros2_foxy_dev/limo/no-gazebo.pref
new file mode 100644
index 0000000..e69a3c6
--- /dev/null
+++ b/ros2_foxy_dev/limo/no-gazebo.pref
@@ -0,0 +1,4 @@
+# Applied only while install-deps.sh is running.
+Package: gazebo* libgazebo* ros-foxy-*gazebo*
+Pin: version *
+Pin-Priority: -1
````

### [ros2_foxy_dev/limo/prepare-workspace.py](ros2_foxy_dev/limo/prepare-workspace.py)

**Aggiunto: righe 1–76.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683).

Seleziona i pacchetti fisici e rimuove nei manifest preparati `limo_car`,
le dipendenze Gazebo e, su richiesta, la camera. Lo script conserva anche
la modalità di copia completa usata inizialmente, ma dopo [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a)
`build-robot.sh` non la usa più: `install-deps.sh` lo invoca con
`--manifests-only` per rosdep e la build opera sui sorgenti originali.

````diff
diff --git a/ros2_foxy_dev/limo/prepare-workspace.py b/ros2_foxy_dev/limo/prepare-workspace.py
new file mode 100644
index 0000000..a8d4d00
--- /dev/null
+++ b/ros2_foxy_dev/limo/prepare-workspace.py
@@ -0,0 +1,76 @@
+#!/usr/bin/env python3
+"""Stage physical robot packages without editing the original workspace."""
+import argparse
+from pathlib import Path
+import shutil
+import xml.etree.ElementTree as ET
+
+PACKAGES = {
+    "limo_msgs": "limo_ros2/limo_msgs",
+    "limo_base": "limo_ros2/limo_base",
+    "limo_bringup": "limo_ros2/limo_bringup",
+    "limo_description": "limo_ros2/limo_description",
+    "custom_start": "custom_start",
+    "ydlidar_ros2_driver": "ydlidar_ros2_driver",
+    "astra_camera_msgs": "ros2_astra_camera/astra_camera_msgs",
+    "astra_camera": "ros2_astra_camera/astra_camera",
+}
+CAMERA_PACKAGES = {"astra_camera", "astra_camera_msgs"}
+
+
+def main():
+    parser = argparse.ArgumentParser(description=__doc__)
+    parser.add_argument("source", type=Path)
+    parser.add_argument("destination", type=Path)
+    parser.add_argument("--without-camera", action="store_true")
+    parser.add_argument("--manifests-only", action="store_true")
+    args = parser.parse_args()
+    source = args.source.resolve()
+    destination = args.destination.resolve()
+    if source == destination or source in destination.parents or destination in source.parents:
+        parser.error("Source and destination must be separate directories")
+    selected = {name: source / relative for name, relative in PACKAGES.items()
+                if not args.without_camera or name not in CAMERA_PACKAGES}
+    for name, directory in selected.items():
+        manifest = directory / "package.xml"
+        if not manifest.is_file() or ET.parse(str(manifest)).getroot().findtext("name") != name:
+            parser.error("Missing or invalid source manifest: {}".format(manifest))
+
+    marker = destination / ".limo-generated"
+    if destination.exists() and any(destination.iterdir()) and not marker.is_file():
+        parser.error("Refusing to overwrite a directory not generated by this profile")
+    destination.mkdir(parents=True, exist_ok=True)
+    marker.touch()
+    # Only remove this helper's previous generated package copies.
+    for name in PACKAGES:
+        old = destination / name
+        if old.exists():
+            shutil.rmtree(str(old))
+    for name, directory in selected.items():
+        target = destination / name
+        if args.manifests_only:
+            target.mkdir()
+        else:
+            shutil.copytree(str(directory), str(target), ignore=shutil.ignore_patterns(
+                ".git", "__pycache__", "*.pyc", "build", "install", "log", "COLCON_IGNORE"))
+        tree = ET.parse(str(directory / "package.xml"))
+        package = tree.getroot()
+        for entry in list(package):
+            if not entry.tag.endswith("depend"):
+                continue
+            dependency = (entry.text or "").strip()
+            if dependency == "limo_car" or dependency.startswith("gazebo") or (
+                    args.without_camera and dependency in CAMERA_PACKAGES):
+                package.remove(entry)
+        tree.write(str(target / "package.xml"), encoding="utf-8", xml_declaration=True)
+        if name == "custom_start" and not args.manifests_only:
+            # The source package combines robot and simulator launch resources.
+            (target / "launch/limo_circuit.launch.py").unlink(missing_ok=True)
+            for resource in ("models", "worlds"):
+                if (target / resource).is_dir():
+                    shutil.rmtree(str(target / resource))
+        print("Prepared {}".format(name))
+
+
+if __name__ == "__main__":
+    main()
````

### [ros2_foxy_dev/limo/setup-devices.sh](ros2_foxy_dev/limo/setup-devices.sh)

**Aggiunto: righe 1–15.**

**Origine:** [`2ba3faa`](https://github.com/chullius17/Limo/commit/2ba3faa66b8e5d1b7cbadd02c727f7431961c683).

L'accesso ai dispositivi viene preparato tramite gruppi `dialout`/`video`
e regole udev per UART chassis, adattatori YDLidar e vendor USB Orbbec.
Il nome `/dev/ydlidar` diventa stabile per gli ID previsti; l'adesione ai
gruppi richiede una nuova sessione. Sono requisiti per usare i driver senza
eseguirli come root, non calibrazioni dei sensori.

````diff
diff --git a/ros2_foxy_dev/limo/setup-devices.sh b/ros2_foxy_dev/limo/setup-devices.sh
new file mode 100644
index 0000000..e66ba33
--- /dev/null
+++ b/ros2_foxy_dev/limo/setup-devices.sh
@@ -0,0 +1,15 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+if [[ "$(id -u)" == 0 ]]; then
+  echo "Run as the normal robot user, without sudo in front of the command." >&2
+  exit 1
+fi
+sudo usermod -aG dialout,video "$(id -un)"
+sudo install -m 0644 "$SCRIPT_DIR/99-limo-hardware.rules" /etc/udev/rules.d/99-limo-hardware.rules
+sudo udevadm control --reload-rules
+sudo udevadm trigger --subsystem-match=tty
+sudo udevadm trigger --subsystem-match=usb
+echo "Log out and back in to apply groups. Reconnect the USB lidar and camera."
+echo "Check that /dev/ydlidar identifies your lidar before launching the robot."
````

### [ros2_foxy_dev/limo/setup-shell.sh](ros2_foxy_dev/limo/setup-shell.sh)

**Aggiunto: righe 1–58.**

**Origine:** [`1d296de`](https://github.com/chullius17/Limo/commit/1d296de02cbfd2c2cadda3faafbe4a1004597a56), con percorso dell'install aggiornato in [`c4ab5cc`](https://github.com/chullius17/Limo/commit/c4ab5cc2741409c359c6e565e1ae3f1f9e2eb88a).

`cb_limo` richiama la build robot, `wsp` entra nel workspace e `start` richiama
l'avvio hardware. `setup-shell.sh` inserisce un blocco delimitato in `.bashrc`,
carica Foxy/install/alias e conserva una copia di backup. Rilanciarlo aggiorna
il suo blocco senza moltiplicare le righe di setup. Questa modifica rende
più semplice l'uso quotidiano, senza cambiare il comportamento del driver.

````diff
diff --git a/ros2_foxy_dev/limo/setup-shell.sh b/ros2_foxy_dev/limo/setup-shell.sh
new file mode 100644
index 0000000..0c22c69
--- /dev/null
+++ b/ros2_foxy_dev/limo/setup-shell.sh
@@ -0,0 +1,58 @@
+#!/usr/bin/env bash
+set -euo pipefail
+
+SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
+if [[ $# == 0 ]]; then
+  bashrc_path="$HOME/.bashrc"
+elif [[ $# == 2 && "$1" == --bashrc ]]; then
+  bashrc_path="$2"
+else
+  echo "Usage: bash setup-shell.sh [--bashrc FILE]" >&2
+  exit 2
+fi
+
+python3 - "$SCRIPT_DIR" "$bashrc_path" <<'PY'
+from pathlib import Path
+import shlex
+import shutil
+import sys
+
+script_dir = Path(sys.argv[1]).resolve()
+bashrc = Path(sys.argv[2]).expanduser()
+install_setup = (script_dir / "../../workspace/install/setup.bash").resolve()
+aliases = script_dir / "aliases/limo.bash"
+begin = "# >>> LIMO shell setup >>>"
+end = "# <<< LIMO shell setup <<<"
+block = "\n".join([
+    begin,
+    "if [[ -f /opt/ros/foxy/setup.bash ]]; then",
+    "  source /opt/ros/foxy/setup.bash",
+    "fi",
+    "if [[ -f {} ]]; then".format(shlex.quote(str(install_setup))),
+    "  source {}".format(shlex.quote(str(install_setup))),
+    "fi",
+    "if [[ -f {} ]]; then".format(shlex.quote(str(aliases))),
+    "  source {}".format(shlex.quote(str(aliases))),
+    "fi",
+    end,
+    "",
+])
+original = bashrc.read_text() if bashrc.exists() else ""
+lines = original.splitlines(keepends=True)
+starts = [i for i, line in enumerate(lines) if line.rstrip("\r\n") == begin]
+ends = [i for i, line in enumerate(lines) if line.rstrip("\r\n") == end]
+if not starts and not ends:
+    updated = original + ("\n" if original and not original.endswith("\n") else "") + block
+elif len(starts) == len(ends) == 1 and starts[0] < ends[0]:
+    updated = "".join(lines[:starts[0]]) + block + "".join(lines[ends[0] + 1:])
+else:
+    sys.exit("Invalid LIMO setup markers in {}. No changes made.".format(bashrc))
+
+if updated != original:
+    backup = bashrc.with_name(bashrc.name + ".limo-backup")
+    if bashrc.exists() and not backup.exists() and not backup.is_symlink():
+        shutil.copy2(str(bashrc), str(backup))
+    bashrc.write_text(updated)
+print("LIMO shell setup configured in {}".format(bashrc))
+print("Open a new terminal or run: source {}".format(shlex.quote(str(bashrc))))
+PY
````

### [workspace/.vscode/settings.json](workspace/.vscode/settings.json)

**Aggiunto: righe 1–3.**

````diff
diff --git a/workspace/.vscode/settings.json b/workspace/.vscode/settings.json
new file mode 100644
index 0000000..578fc24
--- /dev/null
+++ b/workspace/.vscode/settings.json
@@ -0,0 +1,3 @@
+{
+    "cmake.sourceDirectory": "${workspaceFolder}/src/limo_ros2/limo_base"
+}
````

### [workspace/src/.gitignore](workspace/src/.gitignore)

**Aggiunto: righe 1–3.**

````diff
diff --git a/workspace/src/.gitignore b/workspace/src/.gitignore
new file mode 100644
index 0000000..4e6e088
--- /dev/null
+++ b/workspace/src/.gitignore
@@ -0,0 +1,3 @@
+/ydlidar_ros2_driver/COLCON_IGNORE
+/ros2_astra_camera/astra_camera/COLCON_IGNORE
+/ros2_astra_camera/astra_camera_msgs/COLCON_IGNORE
````

### [workspace/src/.vscode/settings.json](workspace/src/.vscode/settings.json)

**Aggiunto: righe 1–3.**

````diff
diff --git a/workspace/src/.vscode/settings.json b/workspace/src/.vscode/settings.json
new file mode 100644
index 0000000..20d8a34
--- /dev/null
+++ b/workspace/src/.vscode/settings.json
@@ -0,0 +1,3 @@
+{
+    "cmake.sourceDirectory": "${workspaceFolder}/YDLidar-SDK"
+}
````

### [workspace/src/custom_start/custom_start/__init__.py](workspace/src/custom_start/custom_start/__init__.py)

**Aggiunto: file vuoto.**

````diff
diff --git a/workspace/src/custom_start/custom_start/__init__.py b/workspace/src/custom_start/custom_start/__init__.py
new file mode 100644
index 0000000..e69de29
````

### [workspace/src/custom_start/models/plane_test/materials/scripts/plane.material](workspace/src/custom_start/models/plane_test/materials/scripts/plane.material)

**Aggiunto: righe 1–19.**

````diff
diff --git a/workspace/src/custom_start/models/plane_test/materials/scripts/plane.material b/workspace/src/custom_start/models/plane_test/materials/scripts/plane.material
new file mode 100644
index 0000000..43f30d1
--- /dev/null
+++ b/workspace/src/custom_start/models/plane_test/materials/scripts/plane.material
@@ -0,0 +1,19 @@
+material vrc/circuit
+{
+  technique
+  {
+    pass
+    {
+      ambient 1 1 1 1
+      diffuse 1 1 1 1
+      specular 0 0 0 1 0
+
+      texture_unit
+      {
+        texture circuit.png
+        filtering anisotropic
+        max_anisotropy 16
+      }
+    }
+  }
+}
````

### [workspace/src/custom_start/models/plane_test/model.config](workspace/src/custom_start/models/plane_test/model.config)

**Aggiunto: righe 1–11.**

````diff
diff --git a/workspace/src/custom_start/models/plane_test/model.config b/workspace/src/custom_start/models/plane_test/model.config
new file mode 100644
index 0000000..0af8cc5
--- /dev/null
+++ b/workspace/src/custom_start/models/plane_test/model.config
@@ -0,0 +1,11 @@
+<?xml version="1.0" ?>
+<model>
+    <name>plane_test</name>
+    <version>1.0</version>
+    <sdf version="1.6">model.sdf</sdf>
+    <author>
+        <name>Giulio</name>
+        <email></email>
+    </author>
+    <description>Circuit floor for LIMO simulation</description>
+</model>
\ No newline at end of file
````

### [workspace/src/custom_start/models/plane_test/model.sdf](workspace/src/custom_start/models/plane_test/model.sdf)

**Aggiunto: righe 1–97.**

````diff
diff --git a/workspace/src/custom_start/models/plane_test/model.sdf b/workspace/src/custom_start/models/plane_test/model.sdf
new file mode 100644
index 0000000..25bf706
--- /dev/null
+++ b/workspace/src/custom_start/models/plane_test/model.sdf
@@ -0,0 +1,97 @@
+<?xml version='1.0'?>
+<sdf version='1.6'>
+  <model name='plane_test'>
+    <link name='link'>
+      <inertial>
+        <mass>13.3626</mass>
+        <inertia>
+          <ixx>86.3001</ixx>
+          <ixy>0</ixy>
+          <ixz>0</ixz>
+          <iyy>89.6046</iyy>
+          <iyz>0</iyz>
+          <izz>175.841</izz>
+        </inertia>
+        <pose frame=''>0 0 0 0 -0 0</pose>
+      </inertial>
+      <self_collide>0</self_collide>
+      <enable_wind>0</enable_wind>
+      <kinematic>0</kinematic>
+      <pose frame=''>0.0 0.0 0 0 -0 0</pose>
+      <visual name='visual'>
+        <geometry>
+          <box>
+            <size>12.057045 7.762755 0.20</size>
+          </box>
+        </geometry>
+        <material>
+          <script>
+            <uri>model://plane_test/materials/scripts</uri>
+            <uri>model://plane_test/materials/textures</uri>
+            <name>vrc/circuit</name>
+          </script>
+        </material>
+        <pose frame=''>0 0 0 0 -0 0</pose>
+        <transparency>0</transparency>
+        <cast_shadows>1</cast_shadows>
+      </visual>
+      <collision name='collision'>
+        <laser_retro>0</laser_retro>
+        <max_contacts>10</max_contacts>
+        <pose frame=''>0 0 0 0 -0 0</pose>
+        <geometry>
+          <box>
+            <size>12.057045 7.762755 0.20</size>
+          </box>
+        </geometry>
+        <surface>
+          <friction>
+            <ode>
+              <mu>1</mu>
+              <mu2>1</mu2>
+              <fdir1>0 0 0</fdir1>
+              <slip1>0</slip1>
+              <slip2>0</slip2>
+            </ode>
+            <torsional>
+              <coefficient>1</coefficient>
+              <patch_radius>0</patch_radius>
+              <surface_radius>0</surface_radius>
+              <use_patch_radius>1</use_patch_radius>
+              <ode>
+                <slip>0</slip>
+              </ode>
+            </torsional>
+          </friction>
+          <bounce>
+            <restitution_coefficient>0</restitution_coefficient>
+            <threshold>1e+06</threshold>
+          </bounce>
+          <contact>
+            <collide_without_contact>0</collide_without_contact>
+            <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
+            <collide_bitmask>1</collide_bitmask>
+            <ode>
+              <soft_cfm>0</soft_cfm>
+              <soft_erp>0.2</soft_erp>
+              <kp>1e+13</kp>
+              <kd>1</kd>
+              <max_vel>0.01</max_vel>
+              <min_depth>0</min_depth>
+            </ode>
+            <bullet>
+              <split_impulse>1</split_impulse>
+              <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
+              <soft_cfm>0</soft_cfm>
+              <soft_erp>0.2</soft_erp>
+              <kp>1e+13</kp>
+              <kd>1</kd>
+            </bullet>
+          </contact>
+        </surface>
+      </collision>
+    </link>
+    <static>1</static>   <!-- To make the plane static, set this to 1 -->
+    <allow_auto_disable>1</allow_auto_disable>
+  </model>
+</sdf>
````

### [workspace/src/custom_start/models/walls/model.config](workspace/src/custom_start/models/walls/model.config)

**Aggiunto: righe 1–11.**

````diff
diff --git a/workspace/src/custom_start/models/walls/model.config b/workspace/src/custom_start/models/walls/model.config
new file mode 100644
index 0000000..799a5e8
--- /dev/null
+++ b/workspace/src/custom_start/models/walls/model.config
@@ -0,0 +1,11 @@
+<?xml version="1.0" ?>
+<model>
+    <name>walls</name>
+    <version>1.0</version>
+    <sdf version="1.7">model.sdf</sdf>
+    <author>
+        <name></name>
+        <email></email>
+    </author>
+    <description></description>
+</model>
````

### [workspace/src/custom_start/models/walls/model.sdf](workspace/src/custom_start/models/walls/model.sdf)

**Aggiunto: righe 1–123.**

````diff
diff --git a/workspace/src/custom_start/models/walls/model.sdf b/workspace/src/custom_start/models/walls/model.sdf
new file mode 100644
index 0000000..93ced26
--- /dev/null
+++ b/workspace/src/custom_start/models/walls/model.sdf
@@ -0,0 +1,123 @@
+<?xml version='1.0'?>
+<sdf version='1.7'>
+  <model name='walls'>
+    <pose>-0.065984688 0.032186847 0 0 -0 0</pose>
+    <link name='Wall_0'>
+      <collision name='Wall_0_Collision'>
+        <geometry>
+          <box>
+            <size>8.25825 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <pose>0 0 0.125 0 -0 0</pose>
+      </collision>
+      <visual name='Wall_0_Visual'>
+        <pose>0 0 0.125 0 -0 0</pose>
+        <geometry>
+          <box>
+            <size>8.25825 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <material>
+          <script>
+            <uri>file://media/materials/scripts/gazebo.material</uri>
+            <name>Gazebo/Grey</name>
+          </script>
+          <ambient>1 1 1 1</ambient>
+        </material>
+        <meta>
+          <layer>0</layer>
+        </meta>
+      </visual>
+      <pose>-6.0984 0 0 0 -0 -1.5708</pose>
+    </link>
+    <link name='Wall_2'>
+      <collision name='Wall_2_Collision'>
+        <geometry>
+          <box>
+            <size>12.387375 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <pose>0 0 0.125 0 -0 0</pose>
+      </collision>
+      <visual name='Wall_2_Visual'>
+        <pose>0 0 0.125 0 -0 0</pose>
+        <geometry>
+          <box>
+            <size>12.387375 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <material>
+          <script>
+            <uri>file://media/materials/scripts/gazebo.material</uri>
+            <name>Gazebo/Grey</name>
+          </script>
+          <ambient>1 1 1 1</ambient>
+        </material>
+        <meta>
+          <layer>0</layer>
+        </meta>
+      </visual>
+      <pose>0 -4.0338375 0 0 -0 0</pose>
+    </link>
+    <link name='Wall_4'>
+      <collision name='Wall_4_Collision'>
+        <geometry>
+          <box>
+            <size>8.25825 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <pose>0 0 0.125 0 -0 0</pose>
+      </collision>
+      <visual name='Wall_4_Visual'>
+        <pose>0 0 0.125 0 -0 0</pose>
+        <geometry>
+          <box>
+            <size>8.25825 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <material>
+          <script>
+            <uri>file://media/materials/scripts/gazebo.material</uri>
+            <name>Gazebo/Grey</name>
+          </script>
+          <ambient>1 1 1 1</ambient>
+        </material>
+        <meta>
+          <layer>0</layer>
+        </meta>
+      </visual>
+      <pose>6.0984 0 0 0 -0 1.5708</pose>
+    </link>
+    <link name='Wall_5'>
+      <collision name='Wall_5_Collision'>
+        <geometry>
+          <box>
+            <size>12.387375 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <pose>0 0 0.125 0 -0 0</pose>
+      </collision>
+      <visual name='Wall_5_Visual'>
+        <pose>0 0 0.125 0 -0 0</pose>
+        <geometry>
+          <box>
+            <size>12.387375 0.190575 0.25</size>
+          </box>
+        </geometry>
+        <material>
+          <script>
+            <uri>file://media/materials/scripts/gazebo.material</uri>
+            <name>Gazebo/Grey</name>
+          </script>
+          <ambient>1 1 1 1</ambient>
+        </material>
+        <meta>
+          <layer>0</layer>
+        </meta>
+      </visual>
+      <pose>0 4.0338375 0 0 -0 3.14159</pose>
+    </link>
+    <static>1</static>
+  </model>
+</sdf>
````

### [workspace/src/custom_start/package.xml](workspace/src/custom_start/package.xml)

**Aggiunto: righe 1–32.**

````diff
diff --git a/workspace/src/custom_start/package.xml b/workspace/src/custom_start/package.xml
new file mode 100644
index 0000000..b686477
--- /dev/null
+++ b/workspace/src/custom_start/package.xml
@@ -0,0 +1,32 @@
+<?xml version="1.0"?>
+<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
+<package format="3">
+  <name>custom_start</name>
+  <version>0.0.0</version>
+  <description>Launch the simulated or physical LIMO Ackermann robot.</description>
+  <maintainer email="root@todo.todo">root</maintainer>
+  <license>Apache-2.0</license>
+
+  <exec_depend>ament_index_python</exec_depend>
+  <exec_depend>astra_camera</exec_depend>
+  <exec_depend>gazebo_ros</exec_depend>
+  <exec_depend>launch</exec_depend>
+  <exec_depend>launch_ros</exec_depend>
+  <exec_depend>python3-yaml</exec_depend>
+  <exec_depend>limo_base</exec_depend>
+  <exec_depend>limo_bringup</exec_depend>
+  <exec_depend>limo_car</exec_depend>
+  <exec_depend>robot_localization</exec_depend>
+  <exec_depend>rviz2</exec_depend>
+  <exec_depend>tf2_ros</exec_depend>
+  <exec_depend>ydlidar_ros2_driver</exec_depend>
+
+  <test_depend>ament_copyright</test_depend>
+  <test_depend>ament_flake8</test_depend>
+  <test_depend>ament_pep257</test_depend>
+  <test_depend>python3-pytest</test_depend>
+
+  <export>
+    <build_type>ament_python</build_type>
+  </export>
+</package>
````

### [workspace/src/custom_start/resource/custom_start](workspace/src/custom_start/resource/custom_start)

**Aggiunto: file vuoto.**

````diff
diff --git a/workspace/src/custom_start/resource/custom_start b/workspace/src/custom_start/resource/custom_start
new file mode 100644
index 0000000..e69de29
````

### [workspace/src/custom_start/setup.cfg](workspace/src/custom_start/setup.cfg)

**Aggiunto: righe 1–4.**

````diff
diff --git a/workspace/src/custom_start/setup.cfg b/workspace/src/custom_start/setup.cfg
new file mode 100644
index 0000000..bb33116
--- /dev/null
+++ b/workspace/src/custom_start/setup.cfg
@@ -0,0 +1,4 @@
+[develop]
+script_dir=$base/lib/custom_start
+[install]
+install_scripts=$base/lib/custom_start
````

### [workspace/src/custom_start/setup.py](workspace/src/custom_start/setup.py)

**Aggiunto: righe 1–42.**

````diff
diff --git a/workspace/src/custom_start/setup.py b/workspace/src/custom_start/setup.py
new file mode 100644
index 0000000..f494308
--- /dev/null
+++ b/workspace/src/custom_start/setup.py
@@ -0,0 +1,42 @@
+import os
+from glob import glob
+
+from setuptools import find_packages, setup
+
+package_name = 'custom_start'
+
+setup(
+    name=package_name,
+    version='0.0.0',
+    packages=find_packages(exclude=['test']),
+    data_files=[
+        ('share/ament_index/resource_index/packages',
+            ['resource/' + package_name]),
+        ('share/' + package_name, ['package.xml']),
+        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
+        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
+        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
+        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
+        # Install all files contained in the nested Gazebo model directories.
+        *[
+            (os.path.join('share', package_name, os.path.dirname(path)), [path])
+            for path in glob('models/**/*', recursive=True)
+            if os.path.isfile(path)
+        ],
+    ],
+    install_requires=['setuptools'],
+    zip_safe=True,
+    maintainer='root',
+    maintainer_email='root@todo.todo',
+    description='Launch the simulated or physical LIMO Ackermann robot.',
+    license='Apache-2.0',
+    extras_require={
+        'test': [
+            'pytest',
+        ],
+    },
+    entry_points={
+        'console_scripts': [
+        ],
+    },
+)
````

### [workspace/src/custom_start/test/test_copyright.py](workspace/src/custom_start/test/test_copyright.py)

**Aggiunto: righe 1–25.**

````diff
diff --git a/workspace/src/custom_start/test/test_copyright.py b/workspace/src/custom_start/test/test_copyright.py
new file mode 100644
index 0000000..97a3919
--- /dev/null
+++ b/workspace/src/custom_start/test/test_copyright.py
@@ -0,0 +1,25 @@
+# Copyright 2015 Open Source Robotics Foundation, Inc.
+#
+# Licensed under the Apache License, Version 2.0 (the "License");
+# you may not use this file except in compliance with the License.
+# You may obtain a copy of the License at
+#
+#     http://www.apache.org/licenses/LICENSE-2.0
+#
+# Unless required by applicable law or agreed to in writing, software
+# distributed under the License is distributed on an "AS IS" BASIS,
+# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
+# See the License for the specific language governing permissions and
+# limitations under the License.
+
+from ament_copyright.main import main
+import pytest
+
+
+# Remove the `skip` decorator once the source file(s) have a copyright header
+@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
+@pytest.mark.copyright
+@pytest.mark.linter
+def test_copyright():
+    rc = main(argv=['.', 'test'])
+    assert rc == 0, 'Found errors'
````

### [workspace/src/custom_start/test/test_flake8.py](workspace/src/custom_start/test/test_flake8.py)

**Aggiunto: righe 1–25.**

````diff
diff --git a/workspace/src/custom_start/test/test_flake8.py b/workspace/src/custom_start/test/test_flake8.py
new file mode 100644
index 0000000..27ee107
--- /dev/null
+++ b/workspace/src/custom_start/test/test_flake8.py
@@ -0,0 +1,25 @@
+# Copyright 2017 Open Source Robotics Foundation, Inc.
+#
+# Licensed under the Apache License, Version 2.0 (the "License");
+# you may not use this file except in compliance with the License.
+# You may obtain a copy of the License at
+#
+#     http://www.apache.org/licenses/LICENSE-2.0
+#
+# Unless required by applicable law or agreed to in writing, software
+# distributed under the License is distributed on an "AS IS" BASIS,
+# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
+# See the License for the specific language governing permissions and
+# limitations under the License.
+
+from ament_flake8.main import main_with_errors
+import pytest
+
+
+@pytest.mark.flake8
+@pytest.mark.linter
+def test_flake8():
+    rc, errors = main_with_errors(argv=[])
+    assert rc == 0, \
+        'Found %d code style errors / warnings:\n' % len(errors) + \
+        '\n'.join(errors)
````

### [workspace/src/custom_start/test/test_pep257.py](workspace/src/custom_start/test/test_pep257.py)

**Aggiunto: righe 1–23.**

````diff
diff --git a/workspace/src/custom_start/test/test_pep257.py b/workspace/src/custom_start/test/test_pep257.py
new file mode 100644
index 0000000..b234a38
--- /dev/null
+++ b/workspace/src/custom_start/test/test_pep257.py
@@ -0,0 +1,23 @@
+# Copyright 2015 Open Source Robotics Foundation, Inc.
+#
+# Licensed under the Apache License, Version 2.0 (the "License");
+# you may not use this file except in compliance with the License.
+# You may obtain a copy of the License at
+#
+#     http://www.apache.org/licenses/LICENSE-2.0
+#
+# Unless required by applicable law or agreed to in writing, software
+# distributed under the License is distributed on an "AS IS" BASIS,
+# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
+# See the License for the specific language governing permissions and
+# limitations under the License.
+
+from ament_pep257.main import main
+import pytest
+
+
+@pytest.mark.linter
+@pytest.mark.pep257
+def test_pep257():
+    rc = main(argv=['.', 'test'])
+    assert rc == 0, 'Found code style errors / warnings'
````

### [workspace/src/custom_start/worlds/limo_circuit_world.world](workspace/src/custom_start/worlds/limo_circuit_world.world)

**Aggiunto: righe 1–34.**

````diff
diff --git a/workspace/src/custom_start/worlds/limo_circuit_world.world b/workspace/src/custom_start/worlds/limo_circuit_world.world
new file mode 100644
index 0000000..8c9dfc0
--- /dev/null
+++ b/workspace/src/custom_start/worlds/limo_circuit_world.world
@@ -0,0 +1,34 @@
+<?xml version="1.0" ?>
+<sdf version="1.5">
+  <world name="default">
+    <scene>
+      <ambient>0.4 0.4 0.4 1</ambient>
+      <background>0.7 0.7 0.7 1</background>
+    </scene>
+
+    <light name="sun" type="directional">
+      <pose>0 0 10 0 0 0</pose>
+      <cast_shadows>true</cast_shadows>
+      <diffuse>0.8 0.8 0.8 1</diffuse>
+      <specular>0.2 0.2 0.2 1</specular>
+      <attenuation>
+        <range>1000</range>
+        <constant>0.9</constant>
+        <linear>0.01</linear>
+        <quadratic>0.001</quadratic>
+      </attenuation>
+      <direction>-0.5 0.1 -0.9</direction>
+    </light>
+
+    <include>
+      <uri>model://plane_test</uri>
+      <pose>0 0 0 0 0 0</pose>
+    </include>
+
+    <include>
+      <uri>model://walls</uri>
+      <pose>0 0 0 0 0 0</pose>
+    </include>
+
+  </world>
+</sdf>
````

### [workspace/src/limo_ros2/limo_base/test/test_steering_serial.py](workspace/src/limo_ros2/limo_base/test/test_steering_serial.py)

**Aggiunto: righe 1–155.**

**Origine:** [`d56c5d7`](https://github.com/chullius17/Limo/commit/d56c5d733f25b27a234b062b07183b718785a28d).

Il test avvia il driver compilato su uno pseudo-terminale in un dominio ROS
isolato e legge i pacchetti seriali effettivamente emessi. Copre 27 comandi,
6 feedback, 3 rifiuti di modifica runtime e 8 calibrazioni invalide,
con scale originali, nuove e asimmetriche. Non usa la porta del telaio:
verifica la conversione software, senza misurare la risposta fisica dello sterzo.

````diff
diff --git a/workspace/src/limo_ros2/limo_base/test/test_steering_serial.py b/workspace/src/limo_ros2/limo_base/test/test_steering_serial.py
new file mode 100644
index 0000000..7ace246
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_base/test/test_steering_serial.py
@@ -0,0 +1,155 @@
+"""Exercise the real driver on a pseudo-terminal; never open chassis hardware.
+
+Run with the built limo_base executable as the only argument. The test uses
+an isolated ROS domain and verifies the actual 0x111 serial command packets.
+"""
+
+import math
+import os
+from pathlib import Path
+import pty
+import select
+import signal
+import struct
+import subprocess
+import sys
+import tempfile
+import time
+
+# Set before importing/initializing ROS; no publishers enter the robot domain.
+os.environ['ROS_DOMAIN_ID'] = '83'
+os.environ['ROS_LOCALHOST_ONLY'] = '1'
+import rclpy  # noqa: E402
+from geometry_msgs.msg import Twist  # noqa: E402
+from nav_msgs.msg import Odometry  # noqa: E402
+from rcl_interfaces.srv import SetParameters  # noqa: E402
+from rclpy.parameter import Parameter  # noqa: E402
+
+
+def run_cases(binary, left, right):
+    master, slave = pty.openpty()
+    with tempfile.TemporaryDirectory(prefix='limo-steering-test-') as directory:
+        serial = Path(directory) / 'tty'
+        serial.symlink_to(os.ttyname(slave))
+        # Driver prefixes /dev/ to port_name. Resolve to this test's PTY only.
+        port = '../' + str(serial).lstrip('/')
+        log = open(Path(directory) / 'driver.log', 'w+')
+        process = subprocess.Popen([
+            str(binary), '--ros-args', '-r', '__node:=steering_serial_test',
+            '-p', 'port_name:=' + port, '-p', 'motion_mode:=1',
+            '-p', 'pub_odom_tf:=false', '-p', 'use_mcnamu:=false',
+            '-p', 'steering_left_scale:=' + str(left),
+            '-p', 'steering_right_scale:=' + str(right),
+        ], stdout=log, stderr=subprocess.STDOUT)
+        node = rclpy.create_node('steering_test_publisher')
+        publisher = node.create_publisher(Twist, '/cmd_vel', 10)
+        try:
+            deadline = time.monotonic() + 15
+            while publisher.get_subscription_count() == 0:
+                assert process.poll() is None, 'Driver exited during startup'
+                assert time.monotonic() < deadline, 'Driver discovery timeout'
+                rclpy.spin_once(node, timeout_sec=0.05)
+            for v, w, expected_inner in (
+                    (0.5, 1.0, math.atan(0.2 / (0.5 - 0.086))),
+                    (0.5, -1.0, -math.atan(0.2 / (0.5 - 0.086))),
+                    (-0.5, 1.0, -math.atan(0.2 / (0.5 - 0.086))),
+                    (-0.5, -1.0, math.atan(0.2 / (0.5 - 0.086))),
+                    (0.2, 1.0, 0.48869), (0.2, -1.0, -0.48869),
+                    (0.0, 1.0, 0.0), (0.5, 0.0, 0.0), (0.0, 0.0, 0.0)):
+                while select.select([master], [], [], 0)[0]:
+                    os.read(master, 4096)
+                message = Twist()
+                message.linear.x, message.angular.z = v, w
+                publisher.publish(message)
+                buffer = b''
+                deadline = time.monotonic() + 3
+                found = None
+                while time.monotonic() < deadline and found is None:
+                    if not select.select([master], [], [], 0.1)[0]:
+                        continue
+                    buffer += os.read(master, 4096)
+                    while len(buffer) >= 14:
+                        if buffer[:2] != b'\x55\x0e':
+                            buffer = buffer[1:]
+                            continue
+                        packet, buffer = buffer[:14], buffer[14:]
+                        assert sum(packet[4:12]) & 255 == packet[13]
+                        if packet[2:4] == b'\x01\x11':
+                            found = struct.unpack('>hhhh', packet[4:12])
+                            break
+                scale = left if expected_inner > 0 else right
+                expected = (int(v * 1000), 0, 0, int(expected_inner / scale * 1000))
+                assert found is not None, (v, w, 'No serial command')
+                assert found[:3] == expected[:3], (found, expected)
+                assert abs(found[3] - expected[3]) <= 1, (found, expected)
+                print('PASS scales={} cmd={} serial={}'.format((left, right), (v, w), found))
+            feedback = []
+            subscription = node.create_subscription(Odometry, '/odom', feedback.append, 10)
+
+            def send_frame(identifier, payload):
+                os.write(master, b'\x55\x0e' + struct.pack('>H', identifier) +
+                         payload + bytes((0, sum(payload) & 255)))
+
+            for raw_angle, scale in ((100, left), (-100, right)):
+                physical_angle = raw_angle / 1000.0 * scale
+                expected_yaw = math.copysign(
+                    0.5 / (0.2 / math.tan(abs(physical_angle)) + 0.086), raw_angle)
+                deadline = time.monotonic() + 5
+                feedback.clear()
+                while time.monotonic() < deadline:
+                    # Raw chassis mode 1 enables Ackermann feedback decoding.
+                    send_frame(0x211, bytes((0, 1, 0, 105, 0, 0, 1, 0)))
+                    send_frame(0x221, struct.pack('>hhhh', 500, 0, 0, raw_angle))
+                    rclpy.spin_once(node, timeout_sec=0.05)
+                    if feedback and abs(feedback[-1].twist.twist.angular.z - expected_yaw) < 1e-6:
+                        break
+                assert feedback and abs(feedback[-1].twist.twist.angular.z - expected_yaw) < 1e-6
+                print('PASS calibrated feedback angle={} yaw={}'.format(raw_angle, expected_yaw))
+            node.destroy_subscription(subscription)
+            client = node.create_client(SetParameters, '/steering_serial_test/set_parameters')
+            assert client.wait_for_service(timeout_sec=5)
+            request = SetParameters.Request(parameters=[
+                Parameter('steering_left_scale', value=1.5).to_parameter_msg()])
+            future = client.call_async(request)
+            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
+            assert future.done() and not future.result().results[0].successful
+            print('PASS calibration is read-only while running')
+        except BaseException:
+            log.flush()
+            log.seek(0)
+            print(log.read())
+            raise
+        finally:
+            if process.poll() is None:
+                process.send_signal(signal.SIGINT)
+                try:
+                    process.wait(timeout=5)
+                except subprocess.TimeoutExpired:
+                    process.kill()
+                    process.wait()
+            node.destroy_node()
+            log.close()
+            os.close(master)
+            os.close(slave)
+
+
+def main():
+    binary = Path(sys.argv[1]).resolve(strict=True)
+    rclpy.init()
+    try:
+        for left, right in ((2.47, 2.47), (1.0, 1.0), (1.0, 2.47)):
+            run_cases(binary, left, right)
+        for name in ('steering_left_scale', 'steering_right_scale'):
+            for value in ('0.0', '-1.0', '.nan', '.inf'):
+                result = subprocess.run([
+                    str(binary), '--ros-args', '-p', 'port_name:=disabled',
+                    '-p', name + ':=' + value], capture_output=True, timeout=5)
+                assert result.returncode != 0, (name, value)
+                assert b'Steering scales must be finite and >= 1.0' in result.stderr
+        print('PASS 27 serial cases, 6 feedback cases, 3 runtime guards, 8 invalid calibrations')
+    finally:
+        rclpy.shutdown()
+
+
+if __name__ == '__main__':
+    main()
````

### [workspace/src/limo_ros2/limo_car/launch/ackermann.launch.py](workspace/src/limo_ros2/limo_car/launch/ackermann.launch.py)

**Aggiunto: righe 1–55.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/launch/ackermann.launch.py b/workspace/src/limo_ros2/limo_car/launch/ackermann.launch.py
new file mode 100755
index 0000000..582a3ef
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/launch/ackermann.launch.py
@@ -0,0 +1,55 @@
+# Autor: Zhui Li
+# E-Mail: lz554113510@gmail.com
+# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
+# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
+# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
+# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.
+
+import os
+
+from ament_index_python.packages import get_package_share_directory
+
+from launch import LaunchDescription
+from launch.substitutions import LaunchConfiguration
+from launch.actions import DeclareLaunchArgument
+from launch_ros.actions import Node
+
+import xacro
+
+
+def generate_launch_description():
+
+    # um die Zeit zu simulieren
+    use_sim_time = LaunchConfiguration('use_sim_time')
+
+    # finden path für Modell
+    pkg_path = os.path.join(get_package_share_directory('limo_car'))
+    xacro_file = os.path.join(pkg_path, 'gazebo', 'ackermann_with_sensor.xacro')
+    robot_description_config = xacro.process_file(xacro_file)
+
+    # Erstellt ein robot_state_publisher Node
+    params = {'robot_description': robot_description_config.toxml(),
+            'use_sim_time': use_sim_time}
+    node_robot_state_publisher = Node(
+        package='robot_state_publisher',
+        executable='robot_state_publisher',
+        output='screen',
+        parameters=[params]
+    )
+
+    twist_adapter = Node(
+        package='limo_car',
+        executable='gazebo_twist_adapter.py',
+        name='gazebo_twist_adapter',
+        output='screen',
+        parameters=[{'use_sim_time': use_sim_time}],
+    )
+
+    return LaunchDescription([
+        DeclareLaunchArgument(
+            'use_sim_time',
+            default_value='false',
+            description='Use sim time if true'),
+        node_robot_state_publisher,
+        twist_adapter,
+    ])
````

### [workspace/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py](workspace/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py)

**Aggiunto: righe 1–77.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py b/workspace/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py
new file mode 100755
index 0000000..664157e
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/launch/ackermann_gazebo.launch.py
@@ -0,0 +1,77 @@
+# Autor: Zhui Li
+# E-Mail: lz554113510@gmail.com
+# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
+# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
+# um den Launch der Simulation in Gazebo zu ermöglichen. Danach mit der Simulation in Gazebo kann man die weitere
+# Forschung arbeiten. Diese Launch-Datei dient zu Ackermann-Type.
+
+import os
+
+from ament_index_python.packages import get_package_share_directory
+
+
+from launch import LaunchDescription
+from launch.actions import DeclareLaunchArgument
+from launch.actions import IncludeLaunchDescription
+from launch.substitutions import LaunchConfiguration
+from launch.launch_description_sources import PythonLaunchDescriptionSource
+
+from launch_ros.actions import Node
+
+def generate_launch_description():
+
+    # definiert Path für Modell
+    package_name = 'limo_car'
+    world_file_path = 'worlds/empty_world.model'
+    rviz_path = 'rviz/gazebo.rviz'
+
+    pkg_path = os.path.join(get_package_share_directory(package_name))
+    world_path = os.path.join(pkg_path, world_file_path)
+    default_rviz_config_path = os.path.join(pkg_path, rviz_path)
+
+    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
+                                     description='Absolute path to rviz config file')
+    rviz_node = Node(
+        package='rviz2',
+        executable='rviz2',
+        name='rviz2',
+        output='screen',
+        arguments=['-d', LaunchConfiguration('rvizconfig')],
+    )
+
+    # Position dafür, wo die Modelle herstellt werden
+    spawn_x_val = '0.0'
+    spawn_y_val = '0.0'
+    spawn_z_val = '0.0'
+    spawn_yaw_val = '0.0'
+
+    mbot = IncludeLaunchDescription(
+        PythonLaunchDescriptionSource([os.path.join(
+            get_package_share_directory(package_name),'launch', 'ackermann.launch.py'
+        )]), launch_arguments={'use_sim_time': 'true', 'world': world_path}.items()
+    )
+
+    # Einbindung der Gazebo-Startdatei, die im gazebo_ros-Paket enthalten ist
+    gazebo = IncludeLaunchDescription(
+        PythonLaunchDescriptionSource([os.path.join(
+            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
+    )
+
+    # laufen ein leere node aus den gazebo_ros package
+    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
+                        arguments=['-topic', 'robot_description',
+                                   '-entity', 'mbot',
+                                   '-x', spawn_x_val,
+                                   '-y', spawn_y_val,
+                                   '-z', spawn_z_val,
+                                   '-Y', spawn_yaw_val],
+                        output='screen')
+
+
+    return LaunchDescription([
+        mbot,
+        gazebo,
+        spawn_entity,
+        rviz_arg,
+        rviz_node
+    ])
````

### [workspace/src/limo_ros2/limo_car/launch/display_ackermann.launch.py](workspace/src/limo_ros2/limo_car/launch/display_ackermann.launch.py)

**Aggiunto: righe 1–68.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/launch/display_ackermann.launch.py b/workspace/src/limo_ros2/limo_car/launch/display_ackermann.launch.py
new file mode 100755
index 0000000..0afafd1
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/launch/display_ackermann.launch.py
@@ -0,0 +1,68 @@
+# Autor: Zhui Li
+# E-Mail: lz554113510@gmail.com
+# Company: Institut für Intermodale Transport- und Logistiksysteme in Technische Universität Braunschweig
+# Description: Diese py-Datei basiert auf Regeln und definiert Funktionen durch python,
+# um Lauch der Simulation in Rviz ermöglichen. Durch die Visualisierung in Rviz kann man deutlich die Modelle verstehen.
+# Es gibt nur Aufbau und Aussehen von Modell. Diese Launch-Datei dient zu Ackermann-Type.
+
+from ament_index_python.packages import get_package_share_path
+
+from launch import LaunchDescription
+from launch.actions import DeclareLaunchArgument
+from launch.conditions import IfCondition, UnlessCondition
+from launch.substitutions import Command, LaunchConfiguration
+
+from launch_ros.actions import Node
+from launch_ros.parameter_descriptions import ParameterValue
+
+def generate_launch_description():
+    limo_car_path = get_package_share_path('limo_car')
+    # default_model_path = limo_car_path / 'urdf/limo_ackerman_base.xacro'
+    default_model_path = limo_car_path / 'gazebo/ackermann_with_sensor.xacro'
+    default_rviz_config_path = limo_car_path / 'rviz/urdf.rviz'
+
+    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
+                                    description='Flag to enable joint_state_publisher_gui')
+    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
+                                      description='Absolute path to robot urdf file')
+    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
+                                     description='Absolute path to rviz config file')
+
+    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
+                                       value_type=str)
+
+    robot_state_publisher_node = Node(
+        package='robot_state_publisher',
+        executable='robot_state_publisher',
+        parameters=[{'robot_description': robot_description}]
+    )
+
+    joint_state_publisher_node = Node(
+        package='joint_state_publisher',
+        executable='joint_state_publisher',
+        condition=UnlessCondition(LaunchConfiguration('gui'))
+    )
+
+    joint_state_publisher_gui_node = Node(
+        package='joint_state_publisher_gui',
+        executable='joint_state_publisher_gui',
+        condition=IfCondition(LaunchConfiguration('gui'))
+    )
+
+    rviz_node = Node(
+        package='rviz2',
+        executable='rviz2',
+        name='rviz2',
+        output='screen',
+        arguments=['-d', LaunchConfiguration('rvizconfig')]
+    )
+
+    return LaunchDescription([
+        gui_arg,
+        model_arg,
+        rviz_arg,
+        joint_state_publisher_node,
+        joint_state_publisher_gui_node,
+        robot_state_publisher_node,
+        rviz_node
+    ])
\ No newline at end of file
````

### [workspace/src/limo_ros2/limo_car/package.xml](workspace/src/limo_ros2/limo_car/package.xml)

**Aggiunto: righe 1–31.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/package.xml b/workspace/src/limo_ros2/limo_car/package.xml
new file mode 100644
index 0000000..82aaf82
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/package.xml
@@ -0,0 +1,31 @@
+<?xml version="1.0"?>
+<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
+<package format="3">
+  <name>limo_car</name>
+  <version>1.0.0</version>
+  <description>the package for limo_simulation</description>
+  <maintainer email="lz554113510@gmail.com">zhui li</maintainer>
+  <license>Apache-2.0</license>
+
+  <buildtool_depend>ament_cmake</buildtool_depend>
+
+  <test_depend>ament_lint_auto</test_depend>
+  <test_depend>ament_lint_common</test_depend>
+  <test_depend>python3-pytest</test_depend>
+  <test_depend>ament_cmake_pytest</test_depend>
+  <exec_depend>rclpy</exec_depend>
+  <exec_depend>geometry_msgs</exec_depend>
+	
+  <depend>robot_state_publisher</depend>
+  <depend>joint_state_publisher</depend>
+  <depend>joint_state_publisher_gui</depend>
+  <depend>xacro</depend>
+  <depend>rviz2</depend>
+  <depend>gazebo_ros</depend>
+  <depend>gazebo_plugins</depend>
+  <depend>rqt_robot_steering</depend>
+  
+  <export>
+    <build_type>ament_cmake</build_type>
+  </export>
+</package>
````

### [workspace/src/limo_ros2/limo_car/test/test_gazebo_twist_adapter.py](workspace/src/limo_ros2/limo_car/test/test_gazebo_twist_adapter.py)

**Aggiunto: righe 1–85.**

**Origine:** [`045fe7d`](https://github.com/chullius17/Limo/commit/045fe7d8e179251ab9e68ddf2f5d2685188ef030).

Verifica conversione della curvatura in avanti e retromarcia, saturazione,
comandi invalidi, topic del plugin e watchdog. Questi test distinguono il
significato del Twist standard da quello atteso da Gazebo e prevengono
la reintroduzione della conversione diretta sbagliata.

````diff
diff --git a/workspace/src/limo_ros2/limo_car/test/test_gazebo_twist_adapter.py b/workspace/src/limo_ros2/limo_car/test/test_gazebo_twist_adapter.py
new file mode 100644
index 0000000..9cdef6c
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/test/test_gazebo_twist_adapter.py
@@ -0,0 +1,85 @@
+"""Check the yaw-rate contract at the Gazebo Foxy actuator boundary."""
+
+import math
+from pathlib import Path
+import time
+from unittest.mock import Mock
+import xml.etree.ElementTree as ET
+
+import pytest
+
+from geometry_msgs.msg import Twist
+import rclpy
+
+from gazebo_twist_adapter import GazeboTwistAdapter, gazebo_command
+
+
+@pytest.mark.parametrize('velocity', [0.10, 0.25, 0.50, -0.10, -0.25])
+@pytest.mark.parametrize('curvature', [-1.5, 0.0, 1.5])
+def test_plugin_executes_requested_curvature_in_both_directions(velocity, curvature):
+    desired_yaw_rate = velocity * curvature
+    speed, angle = gazebo_command(velocity, desired_yaw_rate, 0.24, math.atan(0.24 / 0.55))
+    # gazebo_ros_ackermann_drive.cpp: target_rot *= copysign(1, target_linear).
+    actual_steering = angle * math.copysign(1.0, speed)
+    actual_yaw_rate = speed * math.tan(actual_steering) / 0.24
+    assert actual_yaw_rate == pytest.approx(desired_yaw_rate)
+    assert actual_yaw_rate / speed == pytest.approx(curvature)
+
+
+def test_old_direct_twist_interface_oversteered_at_cruising_speed():
+    speed, desired_yaw_rate = 0.5, 0.3
+    wrong_yaw_rate = speed * math.tan(desired_yaw_rate) / 0.24
+    assert wrong_yaw_rate > 2 * desired_yaw_rate
+    speed, angle = gazebo_command(speed, desired_yaw_rate, 0.24, math.atan(0.24 / 0.55))
+    assert speed * math.tan(angle) / 0.24 == pytest.approx(desired_yaw_rate)
+
+
+@pytest.mark.parametrize('velocity,yaw_rate', [
+    (0.0, 1.0), (0.0001, -1.0), (float('nan'), 0.0), (0.1, float('inf')),
+])
+def test_stopped_and_nonfinite_commands_stop_the_robot(velocity, yaw_rate):
+    assert gazebo_command(velocity, yaw_rate, 0.24, math.atan(0.24 / 0.55)) == (0.0, 0.0)
+
+
+def test_steering_saturation_respects_simulated_inner_wheel_limit():
+    speed, angle = gazebo_command(0.1, 2.0, 0.24, math.atan(0.24 / 0.55))
+    radius = 0.24 / math.tan(angle)
+    assert radius >= 0.55 - 1e-12
+    assert math.atan(0.24 / (radius - (0.168 + 0.045) / 2)) < math.pi / 6
+    assert speed == 0.1
+
+
+def test_only_the_adapter_topic_reaches_the_gazebo_plugin():
+    model = Path(__file__).resolve().parents[1] / 'gazebo' / 'ackermann.xacro'
+    root = ET.parse(model).getroot()
+    plugin = root.find(".//plugin[@name='ackermann_controller']")
+    assert plugin.findtext('ros/remapping') == 'cmd_vel:=/cmd_vel_gazebo'
+
+
+def test_node_converts_input_and_stops_once_on_timeout(monkeypatch):
+    rclpy.init()
+    node = GazeboTwistAdapter()
+    publisher = Mock()
+    node.publisher = publisher
+    try:
+        now = time.monotonic()
+        monkeypatch.setattr(time, 'monotonic', lambda: now)
+        command = Twist()
+        command.linear.x = 0.5
+        command.angular.z = 0.3
+        node._command(command)
+        converted = publisher.publish.call_args[0][0]
+        assert converted.linear.x == 0.5
+        assert converted.angular.z == pytest.approx(math.atan(0.24 * 0.3 / 0.5))
+        node._check_timeout()
+        assert publisher.publish.call_count == 1
+        now += 0.6
+        node._check_timeout()
+        node._check_timeout()
+        assert publisher.publish.call_count == 2
+        stopped = publisher.publish.call_args[0][0]
+        assert stopped.linear.x == 0.0
+        assert stopped.angular.z == 0.0
+    finally:
+        node.destroy_node()
+        rclpy.shutdown()
````

### [workspace/src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro](workspace/src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro)

**Aggiunto: righe 1–194.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro b/workspace/src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro
new file mode 100644
index 0000000..2714af8
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/urdf/limo_ackerman_base.xacro
@@ -0,0 +1,194 @@
+<?xml version="1.0"?>
+<!--Auto: Zhui Li-->
+<!-- Description: Diese xacro-Datei basiert auf Regeln und definiert Funktionen durch xacro,
+um die Erstellung der Gesamtstruktur des Wagens in Ackerman-Type zu ermöglichen. Die Konstruktion einiger dieser Strukturen erfolgt
+in externen Dateien.-->
+
+<robot name="limo_ackerman" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <!-- assoziiert mit anderen hergestellten Modellen -->
+    <xacro:arg name="robot_namespace" default="/" />
+    <xacro:include filename="$(find limo_car)/urdf/limo_anteil.xacro" />
+    <xacro:include filename="$(find limo_car)/urdf/limo_steering_hinge.xacro" />
+
+    <!-- definiert Value -->
+    <xacro:property name="M_PI" value="3.14159"/>
+
+    <xacro:property name="base_mass" value="2.1557"/>
+    <xacro:property name="base_x_size" value="0.13" />
+    <xacro:property name="base_y_size" value="0.12" />
+    <xacro:property name="base_z_size" value="0.10" />
+<!--    <xacro:property name="base_x_size" value="0.19" />-->
+<!--    <xacro:property name="base_y_size" value="0.31" />-->
+<!--    <xacro:property name="base_z_size" value="0.12" />-->
+
+
+
+    <xacro:property name="wheelbase" value="0.2"/>
+    <xacro:property name="track" value="0.14"/>
+    <xacro:property name="wheel_vertical_offset" value="-0.100998" />
+
+
+    <xacro:property name="wheel_mass" value="0.5" />
+    <xacro:property name="wheel_length" value="0.045" />
+    <xacro:property name="wheel_radius" value="0.045" />
+
+    <!-- definieren die Macro-Function für die Rechnungen der Matrizen für Trägheitsmoment-->
+    <!-- Kugel -->
+    <xacro:macro name="sphere_inertial_matrix" params="m r">
+        <inertial>
+            <origin xyz="0.0 0.0 0.0" />
+            <mass value="${m}" />
+            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
+                     iyy="${2*m*r*r/5}" iyz="0"
+                     izz="${2*m*r*r/5}" />
+        </inertial>
+    </xacro:macro>
+    <!-- Zylinder -->
+    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
+        <inertial>
+            <origin xyz="0.0 0.0 0.0" />
+            <mass value="${m}" />
+            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
+                     iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
+                     izz="${m*r*r/2}" />
+        </inertial>
+    </xacro:macro>
+    <!-- Quader -->
+    <xacro:macro name="box_inertial_matrix" params="m w l h">
+        <inertial>
+            <origin xyz="0.0 0.0 0.0" />
+            <mass value="${m}" />
+            <inertia ixx="${m*(w*w+h*h)/12}" ixy = "0" ixz = "0"
+                     iyy="${m*(h*h+l*l)/12}" iyz = "0"
+                     izz="${m*(w*w+l*l)/12}" />
+        </inertial>
+    </xacro:macro>
+
+
+
+    <!-- -->
+    <!-- definiert ein Nullpunkt, um Modell aufzubauen. Es ist einfach für weitere Darstellungspostion in Simulation-Umgebung -->
+    <link name="base_footprint"/>
+
+    <joint name="base_joint" type="fixed">
+        <parent link="base_footprint"/>
+        <child link="base_link"/>
+        <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
+    </joint>
+
+    <!-- herstellt base_link als Karosserie -->
+    <link name="base_link">
+        <visual>
+            <origin xyz="0 0 -0.15" rpy="0 0 1.57" />
+            <geometry>
+                <mesh filename="package://limo_car/meshes/limo_base.dae" scale="1 1 1"/>
+            </geometry>
+        </visual>
+        <!-- Modell für physikalische Kollision zu modellieren. -->
+        <collision>
+            <origin xyz="0 0 0" rpy="0 0 0" />
+            <geometry>
+                <box size="${base_x_size} ${base_y_size} ${base_z_size}"/>
+            </geometry>
+        </collision>
+        <xacro:box_inertial_matrix  m="${base_mass}" w="${base_x_size}" l="${base_y_size}" h="${base_z_size}" />
+    </link>
+
+<!--    &lt;!&ndash; mit macro-function renchnet die Trägheitsmoment &ndash;&gt;-->
+<!--    <link name="inertial_link">-->
+<!--        <inertial>-->
+<!--            <origin xyz="0.0 0.0 0.0" />-->
+<!--            <mass value="${base_mass}" />-->
+<!--            -->
+<!--            <inertia ixx="0.24" ixy="0" ixz="0"-->
+<!--                     iyy="0.96" iyz="0"-->
+<!--                     izz="0.96" />-->
+<!--        </inertial>-->
+<!--    </link>-->
+<!--    &lt;!&ndash; verbindet link für Trägheitsmonent und Modell &ndash;&gt;-->
+<!--    <joint name="inertial_joint" type="fixed">-->
+<!--        <origin xyz="0 0 0" rpy="0 0 0" />-->
+<!--        <parent link="base_link" />-->
+<!--        <child link="inertial_link" />-->
+<!--    </joint>-->
+
+    <!--  Zwei hinten Räder werden anhand eines externen Modells erstellt und mit dem Körper verbunden-->
+    <link name="hinten_left_wheel_link">
+        <!--        <inertial>-->
+        <!--            <origin xyz="0 0 0" />-->
+        <!--            <mass value="0.5" />-->
+        <!--            <inertia ixx="0.01055" ixy="0" ixz="0" iyy="0.00075" iyz="0" izz="0.01055" /> -->
+        <!--        </inertial>-->
+        <visual>
+            <origin xyz="0 0 0" rpy="0 0 0" />
+            <geometry>
+                <mesh filename="package://limo_car/meshes/limo_wheel.dae" />
+            </geometry>
+        </visual>
+        <collision>
+            <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+            <geometry>
+                <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+            </geometry>
+        </collision>
+        <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}" h="${wheel_length}"/>
+    </link>
+    <joint name="hinten_left_wheel" type="continuous">
+        <parent link="base_link"/>
+        <child link="hinten_left_wheel_link"/>
+        <origin xyz="${-wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
+        <axis xyz="0 1 0"/>
+    </joint>
+
+    <link name="hinten_right_wheel_link">
+        <!--        <inertial>-->
+        <!--            <origin xyz="0 0 0" />-->
+        <!--            <mass value="0.5" />-->
+        <!--            <inertia ixx="0.01055" ixy="0" ixz="0" iyy="0.00075" iyz="0" izz="0.01055" /> -->
+        <!--        </inertial>-->
+        <visual>
+            <origin xyz="0 0 0" rpy="0 0 0" />
+            <geometry>
+                <mesh filename="package://limo_car/meshes/limo_wheel.dae" />
+            </geometry>
+        </visual>
+        <collision>
+            <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+            <geometry>
+                <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+            </geometry>
+        </collision>
+        <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}" h="${wheel_length}"/>
+    </link>
+
+    <joint name="hinten_right_wheel" type="continuous">
+        <parent link="base_link"/>
+        <child link="hinten_right_wheel_link"/>
+        <origin xyz="${-wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="${M_PI} 0 0" />
+        <axis xyz="0 -1 0"/>
+    </joint>
+
+
+    <!-- Instanziierung anderer Teile entsprechend den Makrofunktionen in "limo_anteil.xacro" -->
+    <xacro:limo_laser parent_prefix="base_link" frame_prefix="laser">
+        <origin xyz="0.103 0 -0.034" rpy="0 0 0"/>
+    </xacro:limo_laser>
+
+    <xacro:limo_depth_camera parent_prefix="base_link" frame_prefix="depth_camera">
+        <origin xyz="0.084 0 0.03" rpy="0 0 0"/>
+    </xacro:limo_depth_camera>
+
+    <xacro:limo_imu parent_prefix="base_link" frame_prefix="imu">
+        <origin xyz="0.0 0 -0.1" rpy="0 0 0"/>
+    </xacro:limo_imu>
+
+    <!-- ruft Steuerscharnier mit vorderem Räder auf -->
+    <xacro:limo_left_steering_hinge parent_prefix="base_link" wheel_prefix="left_steering_hinge">
+        <origin xyz="${wheelbase/2} ${track/2} ${wheel_vertical_offset}" rpy="0 0 0" />
+    </xacro:limo_left_steering_hinge>
+
+    <xacro:limo_right_steering_hinge parent_prefix="base_link" wheel_prefix="right_steering_hinge">
+        <origin xyz="${wheelbase/2} ${-track/2} ${wheel_vertical_offset}" rpy="${M_PI} 0 0" />
+    </xacro:limo_right_steering_hinge>
+</robot>
````

### [workspace/src/limo_ros2/limo_car/urdf/limo_anteil.xacro](workspace/src/limo_ros2/limo_car/urdf/limo_anteil.xacro)

**Aggiunto: righe 1–176.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/urdf/limo_anteil.xacro b/workspace/src/limo_ros2/limo_car/urdf/limo_anteil.xacro
new file mode 100644
index 0000000..4c387cb
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/urdf/limo_anteil.xacro
@@ -0,0 +1,176 @@
+<?xml version="1.0"?>
+<!--Auto: Zhui Li-->
+<!-- Description: Diese xacro-Datei basiert auf den Regeln und definiert Funktionen durch xacro,
+um die Erstellung von Teilen des Wagens zu ermöglichen. Durch Macrofunktion dient es einfach beim Aufruf in limo_base.-->
+
+<robot name="limo_anteil" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <xacro:macro name="limo_wheel" params="parent_prefix wheel_prefix reflect *joint_pose">
+        <link name="${wheel_prefix}_wheel_link">
+            <inertial>
+                <origin xyz="0 0 0" />
+                <mass value="0.5" />
+                <inertia ixx="0.01055" ixy="0" ixz="0" iyy="0.00075" iyz="0" izz="0.01055" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="package://limo_car/meshes/limo_wheel.dae" />
+                </geometry>
+            </visual>
+            <collision>
+              <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+              <geometry>
+                  <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+              </geometry>
+            </collision>
+        </link>
+
+        <joint name="${wheel_prefix}_wheel" type="continuous">
+            <parent link="${parent_prefix}"/>
+            <child link="${wheel_prefix}_wheel_link"/>
+            <xacro:insert_block name="joint_pose"/>
+            <axis xyz="0 ${reflect*1} 0"/>
+        </joint>
+    </xacro:macro>
+
+    <!-- Laser_Sensor -->
+    <xacro:macro name="limo_laser" params="parent_prefix frame_prefix *joint_pose">
+        <link name='${frame_prefix}_link'>
+            <pose>0 0 0 0 0 0</pose>
+            <inertial>
+                <mass value="0.1"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                    ixx="1e-6" ixy="0" ixz="0"
+                    iyy="1e-6" iyz="0"
+                    izz="1e-6"
+                />
+            </inertial>
+            <visual name='laser_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <cylinder radius="0.02" length="0.01"/>
+                </geometry>
+                <material name='laser_material'>
+                    <color rgba="0.75 0.75 0.75 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='laser_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <cylinder radius="0.032" length="0.016"/>
+                </geometry>
+            </collision>
+        </link>
+        <joint type="fixed" name="laser_joint">
+            <parent link="${parent_prefix}"/>
+            <child link="${frame_prefix}_link"/>
+            <xacro:insert_block name="joint_pose"/>
+        </joint>
+        <!-- definiert diesen Anteil gleiche Farbe in Gazebo -->
+        <gazebo reference="${frame_prefix}_link">
+            <material>Gazebo/Yellow</material>
+        </gazebo>
+    </xacro:macro>
+
+    <!-- depth_camera -->
+    <xacro:macro name="limo_depth_camera" params="parent_prefix frame_prefix *joint_pose">
+        <link name='${frame_prefix}_link'>
+            <pose>0 0 0 0 0 0</pose>
+            <inertial>
+                <mass value="0.1"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                    ixx="1e-6" ixy="0" ixz="0"
+                    iyy="1e-6" iyz="0"
+                    izz="1e-6"
+                />
+            </inertial>
+            <visual name='depth_camera_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.02 0.06 0.015"/>
+                </geometry>
+                <material name='depth_camera_material'>
+                    <color rgba="0.75 0.75 0.75 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='depth_camera_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.02 0.06 0.015"/>
+                </geometry>
+            </collision>
+        </link>
+    
+        <joint type="fixed" name="depth_camera_joint">
+            <xacro:insert_block name="joint_pose"/>
+            <child link="${frame_prefix}_link"/>
+            <parent link="${parent_prefix}"/>
+        </joint>
+
+        <link name="depth_link"></link>
+
+        <joint name="${frame_prefix}_to_camera_joint" type="fixed">
+            <origin xyz="0 0 0" rpy="${-M_PI/2} 0 ${-M_PI/2} "/>
+            <parent link="${frame_prefix}_link"/>
+            <child link="depth_link"/>
+        </joint>
+
+        <gazebo reference="${frame_prefix}_link">
+            <material>Gazebo/Yellow</material>
+        </gazebo>
+    </xacro:macro>
+
+    <!-- IMU-System(Hardware) -->
+    <xacro:macro name="limo_imu" params="parent_prefix frame_prefix  *joint_pose">
+        <link name='${frame_prefix}_link'>
+            <pose>0 0 0 0 0 0</pose>
+            <inertial>
+                <mass value="0.01"/>
+                <origin xyz="0.0 0 0" rpy="0 0 0"/>
+                <inertia
+                    ixx="1e-7" ixy="0" ixz="0"
+                    iyy="1e-7" iyz="0"
+                    izz="1e-7"
+                />
+            </inertial>
+            <visual name='imu_visual'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.001 0.001 0.001"/>
+                </geometry>
+                <material name='imu_material'>
+                    <color rgba="0.75 0.75 0.75 1" />
+                    <ambient>0.1 0.1 0.1 1</ambient>
+                    <diffuse>0.1 0.1 0.2 1</diffuse>
+                    <specular>0 0 0 0</specular>
+                    <emissive>0 0 0 1</emissive>
+                </material>
+            </visual>
+            <collision name='imu_collision'>
+                <origin xyz="0 0 0" rpy="0 0 0"/>
+                <geometry>
+                    <box size="0.001 0.001 0.001"/>
+                </geometry>
+            </collision>
+        </link>
+        <joint type="fixed" name="imu_joint">
+            <xacro:insert_block name="joint_pose"/>
+            <child link="${frame_prefix}_link"/>
+            <parent link="${parent_prefix}"/>
+        </joint>
+        <gazebo reference="${frame_prefix}_link">
+            <material>Gazebo/Yellow</material>
+        </gazebo>
+    </xacro:macro>
+</robot>
````

### [workspace/src/limo_ros2/limo_car/urdf/limo_steering_hinge.xacro](workspace/src/limo_ros2/limo_car/urdf/limo_steering_hinge.xacro)

**Aggiunto: righe 1–126.**

````diff
diff --git a/workspace/src/limo_ros2/limo_car/urdf/limo_steering_hinge.xacro b/workspace/src/limo_ros2/limo_car/urdf/limo_steering_hinge.xacro
new file mode 100644
index 0000000..83d1ac7
--- /dev/null
+++ b/workspace/src/limo_ros2/limo_car/urdf/limo_steering_hinge.xacro
@@ -0,0 +1,126 @@
+<?xml version="1.0"?>
+<!--Auto: Zhui Li-->
+<!-- Description: Diese xacro-Datei basiert auf den Regeln und definiert Funktionen durch xacro,
+um die Erstellung von Steuerscharnier des Wagens zu ermöglichen. Durch Macrofunktion dient es einfach beim Aufruf in limo_base.-->
+
+<robot name="limo_steering_hinge" xmlns:xacro="http://www.ros.org/wiki/xacro">
+
+    <xacro:macro name="limo_left_steering_hinge" params="parent_prefix wheel_prefix *joint_pose">
+        <link name="${wheel_prefix}">
+            <inertial>
+                <!-- diese Daten kommen aus das Internet -->
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <!-- es ist sehr klein -->
+                    <cylinder length="0.0001" radius="0.0001" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius-0.01}" />
+                </geometry>
+            </collision>
+        </link>
+
+        <joint name="${wheel_prefix}_wheel" type="revolute">
+            <parent link="${parent_prefix}"/>
+            <child link="${wheel_prefix}"/>
+            <xacro:insert_block name="joint_pose"/>
+            <axis xyz="0 0 1" />
+            <limit lower="-0.523598767" upper="0.523598767" effort="5" velocity="0.5" />
+            <dynamics damping="1.0" friction="2.0"/>
+        </joint>
+
+        <link name="front_left_wheel_link">
+            <inertial>
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="package://limo_car/meshes/limo_wheel.dae" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+        </link>
+
+        <joint name="front_left_wheel" type="continuous">
+            <parent link="${wheel_prefix}"/>
+            <child link="front_left_wheel_link"/>
+            <origin xyz="0 0 0" rpy="0 0 0" />
+            <axis xyz="0 1 0"/>
+        </joint>
+    </xacro:macro>
+
+
+    <xacro:macro name="limo_right_steering_hinge" params="parent_prefix wheel_prefix *joint_pose">
+        <link name="${wheel_prefix}">
+            <inertial>
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <cylinder length="0.0001" radius="0.0001" />
+                </geometry>
+            </visual>
+            <collision>
+              <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+              <geometry>
+                  <cylinder length="${wheel_length}" radius="${wheel_radius-0.01}" />
+              </geometry>
+            </collision>
+        </link>
+
+        <joint name="${wheel_prefix}_wheel" type="revolute">
+            <parent link="${parent_prefix}"/>
+            <child link="${wheel_prefix}"/>
+            <xacro:insert_block name="joint_pose"/>
+            <axis xyz="0 0 -1"/>
+            <limit lower="-0.523598767" upper="0.523598767" effort="5" velocity="0.5" />
+            <dynamics damping="1.0" friction="2.0"/>
+        </joint>
+
+        <link name="front_right_wheel_link">
+            <inertial>
+                <mass value="0.25" />
+                <inertia ixx="0.00525" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00525" />
+                <origin xyz="0 0 0" />
+            </inertial>
+            <visual>
+                <origin xyz="0 0 0" rpy="0 0 0" />
+                <geometry>
+                    <mesh filename="package://limo_car/meshes/limo_wheel.dae" />
+                </geometry>
+            </visual>
+            <collision>
+                <origin xyz="0 ${wheel_length/2} 0" rpy="1.57 0 0" />
+                <geometry>
+                    <cylinder length="${wheel_length}" radius="${wheel_radius}" />
+                </geometry>
+            </collision>
+        </link>
+
+        <joint name="front_right_wheel" type="continuous">
+            <parent link="${wheel_prefix}"/>
+            <child link="front_right_wheel_link"/>
+            <origin xyz="0 0 0" rpy="0 0  0" />
+            <axis xyz="0 -1 0"/>
+        </joint>
+    </xacro:macro>
+</robot>
````

## Driver importati: righe aggiunte nei punti di integrazione

Anche questi file sono interamente aggiunti rispetto ad AgileX. Qui sono riportati i blocchi di integrazione; il file `.patch` contiene tutto il codice. Non è un confronto con le repository originali YDLidar o Orbbec.

### [workspace/src/ydlidar_ros2_driver/src/scan_ranges.hpp](workspace/src/ydlidar_ros2_driver/src/scan_ranges.hpp)

**Righe locali 1–28.** Normalizzazione dei range invalidi dopo il riempimento dello scan.

**Origine:** [`0aa6c66`](https://github.com/chullius17/Limo/commit/0aa6c663481b24515d14d82ff93f44c1492d0eec).

La normalizzazione copre sia campioni invalidi ricevuti dall'SDK sia bin angolari
rimasti vuoti. Con l'opzione attiva, zeri, negativi, non finiti e fuori intervallo
diventano `+inf`; le misure valide e i loro indici restano invariati.
Questo corregge i punti falsi all'origine usati nel matcher SLAM.

````diff
--- /dev/null
+++ b/workspace/src/ydlidar_ros2_driver/src/scan_ranges.hpp
@@ -0,0 +1,28 @@ estratto delle righe aggiunte
+#ifndef YDLIDAR_ROS2_DRIVER_SCAN_RANGES_HPP_
+#define YDLIDAR_ROS2_DRIVER_SCAN_RANGES_HPP_
+
+#include <cmath>
+#include <limits>
+#include <vector>
+
+namespace ydlidar_ros2_driver
+{
+// Apply this after binning so unfilled bins and invalid SDK samples are covered.
+inline void normalizeInvalidRanges(
+  std::vector<float> & ranges, float range_min, float range_max,
+  bool invalid_range_is_inf)
+{
+  if (!invalid_range_is_inf) {
+    return;
+  }
+  for (float & range : ranges) {
+    if (!std::isfinite(range) || range <= 0.0f ||
+      range < range_min || range > range_max)
+    {
+      range = std::numeric_limits<float>::infinity();
+    }
+  }
+}
+}  // namespace ydlidar_ros2_driver
+
+#endif  // YDLIDAR_ROS2_DRIVER_SCAN_RANGES_HPP_
````

### [workspace/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp](workspace/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp)

**Righe locali 243–249.** Applicazione della normalizzazione nel nodo lidar.

**Origine:** [`0aa6c66`](https://github.com/chullius17/Limo/commit/0aa6c663481b24515d14d82ff93f44c1492d0eec).

La chiamata è collocata dopo il riempimento dei bin: normalizzare soltanto i
campioni in ingresso avrebbe lasciato a zero le celle non riempite.
Il driver prima leggeva il parametro, ma non applicava questo trattamento.

````diff
--- /dev/null
+++ b/workspace/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp
@@ -0,0 +243,7 @@ estratto delle righe aggiunte
+      }
+      // Foxy Karto can match finite zero readings as points at the sensor
+      // origin. Honor the configured invalid-range representation, including
+      // angular bins that received no SDK sample.
+      ydlidar_ros2_driver::normalizeInvalidRanges(
+        scan_msg->ranges, scan_msg->range_min, scan_msg->range_max,
+        invalid_range_is_inf);
````

### [workspace/src/ydlidar_ros2_driver/CMakeLists.txt](workspace/src/ydlidar_ros2_driver/CMakeLists.txt)

**Righe locali 25–37.** Selezione esplicita della libreria SDK vicina al package config.

**Origine:** [`caf9d9d`](https://github.com/chullius17/Limo/commit/caf9d9dbc6407b5b59641d040756423e257a7120), 28 settembre.

La libreria viene cercata vicino al package config dell'SDK selezionato,
con `NO_DEFAULT_PATH`. Questo evita di compilare contro gli header locali
e poi collegare un vecchio SDK trovato in `/usr/local/lib`.
Se la libreria coerente manca, la configurazione fallisce esplicitamente.

````diff
--- /dev/null
+++ b/workspace/src/ydlidar_ros2_driver/CMakeLists.txt
@@ -0,0 +25,13 @@ estratto delle righe aggiunte
+find_package(ydlidar_sdk REQUIRED)
+
+# The SDK config exports headers but leaves its library directory empty.
+# Resolve the archive next to the config file so we do not silently link a
+# stale system-wide copy from /usr/local/lib.
+find_library(YDLIDAR_SDK_LIBRARY
+  NAMES ydlidar_sdk
+  HINTS "${ydlidar_sdk_DIR}/../.."
+  NO_DEFAULT_PATH)
+if(NOT YDLIDAR_SDK_LIBRARY)
+  message(FATAL_ERROR "Could not find the YDLidar SDK library next to ${ydlidar_sdk_DIR}")
+endif()
+set(YDLIDAR_SDK_LIBRARIES "${YDLIDAR_SDK_LIBRARY};pthread;rt")
````

### [workspace/src/YDLidar-SDK/src/CYdLidar.cpp](workspace/src/YDLidar-SDK/src/CYdLidar.cpp)

**Righe locali 768–778.** Warning limitato alla prima occorrenza e poi ogni 500 scan sovradimensionati.

**Origine:** [`caf9d9d`](https://github.com/chullius17/Limo/commit/caf9d9dbc6407b5b59641d040756423e257a7120).

Il warning per uno scan più grande della dimensione fissata viene emesso alla
prima occorrenza e poi ogni 500 occorrenze. Il ridimensionamento rimane attivo:
questa modifica riduce il rumore nei log, senza eliminare i punti eccedenti
alla sorgente o dimostrare un miglioramento dello SLAM.

````diff
--- /dev/null
+++ b/workspace/src/YDLidar-SDK/src/CYdLidar.cpp
@@ -0,0 +768,11 @@ estratto delle righe aggiunte
+        // Keep fixed-size scans for downstream consumers, but avoid flooding
+        // the console when the motor consistently returns a few extra points.
+        static unsigned int oversized_scan_count = 0;
+        ++oversized_scan_count;
+        if (oversized_scan_count == 1 || oversized_scan_count % 500 == 0)
+        {
+          warn("[YDLIDAR]: Real points %lu > fixed points %d "
+               "(rate-limited, occurrences: %u)",
+               count, all_node_count, oversized_scan_count);
+        }
+      	outscan.points.resize(all_node_count);
````

### [workspace/src/ros2_astra_camera/astra_camera/CMakeLists.txt](workspace/src/ros2_astra_camera/astra_camera/CMakeLists.txt)

**Righe locali 223–234.** Installazione del runtime anche con il nome richiesto dal SONAME.

**Origine:** [`cd0525c`](https://github.com/chullius17/Limo/commit/cd0525ce95cadaf772c33e007e93480c72774876), 7 settembre.

Il runtime Orbbec espone il SONAME `libOpenNI2.so.0`: installare soltanto
`libOpenNI2.so` poteva causare la selezione di una libreria di sistema diversa.
La seconda installazione fornisce il nome richiesto dal loader.
Come spiega il README robot, occorrono anche `liborbbec.so` e i file di
configurazione della redistribuzione adatta alla CPU; la sola OpenNI2 generica
non sostituisce il runtime del produttore. `openni2_redist` manca dal checkout.

````diff
--- /dev/null
+++ b/workspace/src/ros2_astra_camera/astra_camera/CMakeLists.txt
@@ -0,0 +223,12 @@ estratto delle righe aggiunte
+install(FILES openni2_redist/${HOST_PLATFORM}/libOpenNI2.so
+  DESTINATION lib/
+  )
+
+# The bundled OpenNI2 library advertises the SONAME libOpenNI2.so.0. Install
+# that runtime name as well, otherwise the loader can silently select an
+# incompatible system OpenNI2 library even though this package linked against
+# the bundled Orbbec build.
+install(FILES openni2_redist/${HOST_PLATFORM}/libOpenNI2.so
+  DESTINATION lib/
+  RENAME libOpenNI2.so.0
+  )
````

### [workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_node_factory.cpp](workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_node_factory.cpp)

**Righe locali 225–226.** Registrazione del componente nella translation unit del driver.

**Origine:** [`0e500b0`](https://github.com/chullius17/Limo/commit/0e500b0d5978db81caa0fd1b80ab808fcc853b21), 12 settembre.

La registrazione del componente è spostata dall'header al file di implementazione,
evitando di replicare la registrazione nelle translation unit che includono
l'header. Fa parte delle correzioni per il caricamento del driver camera.

````diff
--- /dev/null
+++ b/workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_node_factory.cpp
@@ -0,0 +225,2 @@ estratto delle righe aggiunte
+
+RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::OBCameraNodeFactory)
````

## Altri file aggiunti: posizione esatta nel diff completo

Per ogni file sotto, il link apre il relativo blocco nel diff completo. Le mesh copiate identiche sono indicate con `copy from` / `copy to`; le altre risorse binarie sono incluse come `GIT binary patch`.

### ros2_foxy_dev/lab

- [ros2_foxy_dev/lab/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:668)

### ros2_foxy_dev/lab_wsl

- [ros2_foxy_dev/lab_wsl/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:1026)

### ros2_foxy_dev/limo

- [ros2_foxy_dev/limo/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:1659)
- [ros2_foxy_dev/limo/README_fr.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:2093)

### workspace/.gitkeep

- [workspace/.gitkeep](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:2976)

### workspace/src/YDLidar-SDK

- [workspace/src/YDLidar-SDK/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:3010)
- [workspace/src/YDLidar-SDK/Doxyfile](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:3255)
- [workspace/src/YDLidar-SDK/LICENSE.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:5740)
- [workspace/src/YDLidar-SDK/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:5996)
- [workspace/src/YDLidar-SDK/cmake/FindPackage.cmake.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6077)
- [workspace/src/YDLidar-SDK/cmake/PackageConfig.cmake.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6093)
- [workspace/src/YDLidar-SDK/cmake/PackageConfigVersion.cmake.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6123)
- [workspace/src/YDLidar-SDK/cmake/PkgConfig.pc.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6146)
- [workspace/src/YDLidar-SDK/cmake/cmake_uninstall.cmake.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6163)
- [workspace/src/YDLidar-SDK/cmake/common/ydlidar_base.cmake](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6195)
- [workspace/src/YDLidar-SDK/cmake/common/ydlidar_parse.cmake](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6589)
- [workspace/src/YDLidar-SDK/cmake/install_package.cmake](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6666)
- [workspace/src/YDLidar-SDK/cmake/script_show_final_summary.cmake](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:6968)
- [workspace/src/YDLidar-SDK/core/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7068)
- [workspace/src/YDLidar-SDK/core/base/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7082)
- [workspace/src/YDLidar-SDK/core/base/datatype.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7099)
- [workspace/src/YDLidar-SDK/core/base/locker.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7253)
- [workspace/src/YDLidar-SDK/core/base/thread.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7618)
- [workspace/src/YDLidar-SDK/core/base/timer.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7803)
- [workspace/src/YDLidar-SDK/core/base/timer.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7866)
- [workspace/src/YDLidar-SDK/core/base/typedef.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7924)
- [workspace/src/YDLidar-SDK/core/base/utils.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7947)
- [workspace/src/YDLidar-SDK/core/base/v8stdint.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:7971)
- [workspace/src/YDLidar-SDK/core/base/ydlidar.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:8121)
- [workspace/src/YDLidar-SDK/core/common/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:8321)
- [workspace/src/YDLidar-SDK/core/common/ChannelDevice.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:8332)
- [workspace/src/YDLidar-SDK/core/common/DriverInterface.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:8472)
- [workspace/src/YDLidar-SDK/core/common/YdDataStream.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9115)
- [workspace/src/YDLidar-SDK/core/common/YdDataStream.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9308)
- [workspace/src/YDLidar-SDK/core/common/ydlidar_datatype.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9418)
- [workspace/src/YDLidar-SDK/core/common/ydlidar_def.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9547)
- [workspace/src/YDLidar-SDK/core/common/ydlidar_def.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9592)
- [workspace/src/YDLidar-SDK/core/common/ydlidar_help.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:9844)
- [workspace/src/YDLidar-SDK/core/common/ydlidar_protocol.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:11035)
- [workspace/src/YDLidar-SDK/core/json/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:11587)
- [workspace/src/YDLidar-SDK/core/json/cJSON.c](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:11598)
- [workspace/src/YDLidar-SDK/core/json/cJSON.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:14723)
- [workspace/src/YDLidar-SDK/core/math/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15029)
- [workspace/src/YDLidar-SDK/core/math/angles.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15040)
- [workspace/src/YDLidar-SDK/core/network/ActiveSocket.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15343)
- [workspace/src/YDLidar-SDK/core/network/ActiveSocket.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15651)
- [workspace/src/YDLidar-SDK/core/network/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15757)
- [workspace/src/YDLidar-SDK/core/network/PassiveSocket.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:15773)
- [workspace/src/YDLidar-SDK/core/network/PassiveSocket.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:16094)
- [workspace/src/YDLidar-SDK/core/network/SimpleSocket.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:16228)
- [workspace/src/YDLidar-SDK/core/network/SimpleSocket.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:17748)
- [workspace/src/YDLidar-SDK/core/network/StatTimer.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18445)
- [workspace/src/YDLidar-SDK/core/serial/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18601)
- [workspace/src/YDLidar-SDK/core/serial/common.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18617)
- [workspace/src/YDLidar-SDK/core/serial/impl/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18671)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18685)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/list_ports_linux.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:18695)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/lock.c](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:19032)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/lock.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:19834)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/unix.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:20081)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/unix_serial.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:20114)
- [workspace/src/YDLidar-SDK/core/serial/impl/unix/unix_serial.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:21847)
- [workspace/src/YDLidar-SDK/core/serial/impl/windows/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:22015)
- [workspace/src/YDLidar-SDK/core/serial/impl/windows/list_ports_win.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:22026)
- [workspace/src/YDLidar-SDK/core/serial/impl/windows/win.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:22228)
- [workspace/src/YDLidar-SDK/core/serial/impl/windows/win_serial.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:22244)
- [workspace/src/YDLidar-SDK/core/serial/impl/windows/win_serial.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:23327)
- [workspace/src/YDLidar-SDK/core/serial/serial.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:23508)
- [workspace/src/YDLidar-SDK/core/serial/serial.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:23923)
- [workspace/src/YDLidar-SDK/csharp/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24639)
- [workspace/src/YDLidar-SDK/csharp/examples/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24682)
- [workspace/src/YDLidar-SDK/csharp/examples/Program.cs](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24699)
- [workspace/src/YDLidar-SDK/csharp/ydlidar_sdk.i](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24750)
- [workspace/src/YDLidar-SDK/doc/Dataset.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24819)
- [workspace/src/YDLidar-SDK/doc/Diagram.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24859)
- [workspace/src/YDLidar-SDK/doc/FAQs/General_FAQs.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24916)
- [workspace/src/YDLidar-SDK/doc/FAQs/General_FAQs_cn.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24954)
- [workspace/src/YDLidar-SDK/doc/FAQs/Hardware_FAQs.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24970)
- [workspace/src/YDLidar-SDK/doc/FAQs/Hardware_FAQs_cn.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:24984)
- [workspace/src/YDLidar-SDK/doc/FAQs/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25003)
- [workspace/src/YDLidar-SDK/doc/FAQs/Software_FAQs.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25017)
- [workspace/src/YDLidar-SDK/doc/FAQs/Software_FAQs_cn.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25036)
- [workspace/src/YDLidar-SDK/doc/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25052)
- [workspace/src/YDLidar-SDK/doc/Tutorials.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25083)
- [workspace/src/YDLidar-SDK/doc/YDLIDAR_SDK_API_for_Developers.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:25115)
- [workspace/src/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.html](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:26578)
- [workspace/src/YDLidar-SDK/doc/YDLidar-SDK-Communication-Protocol.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27170)
- [workspace/src/YDLidar-SDK/doc/howto/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27523)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_build_and_debug_using_vscode.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27547)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_build_and_install.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27567)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_create_a_csharp_project.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27760)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_create_a_pull.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27801)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_create_a_udev_rules.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27860)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_gerenrate_vs_project_by_cmake.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27931)
- [workspace/src/YDLidar-SDK/doc/howto/how_to_solve_slow_pull_from_cn.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27954)
- [workspace/src/YDLidar-SDK/doc/howto/images/cpack.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:27999)
- [workspace/src/YDLidar-SDK/doc/howto/images/csharp_build.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:29463)
- [workspace/src/YDLidar-SDK/doc/howto/images/csharp_exception.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:30464)
- [workspace/src/YDLidar-SDK/doc/howto/images/csharp_library.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:32096)
- [workspace/src/YDLidar-SDK/doc/howto/images/csharp_project.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:32711)
- [workspace/src/YDLidar-SDK/doc/howto/images/csharp_running.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:33465)
- [workspace/src/YDLidar-SDK/doc/howto/images/sdk_init.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:34640)
- [workspace/src/YDLidar-SDK/doc/howto/images/sdk_scanning.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:34987)
- [workspace/src/YDLidar-SDK/doc/howto/images/sequence.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:36160)
- [workspace/src/YDLidar-SDK/doc/howto/images/ttyUSB0.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:36891)
- [workspace/src/YDLidar-SDK/doc/howto/images/ttyUSB1.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:37736)
- [workspace/src/YDLidar-SDK/doc/howto/images/vscode/run_tasks.jpg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:39170)
- [workspace/src/YDLidar-SDK/doc/howto/images/vscode/tasks.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:56587)
- [workspace/src/YDLidar-SDK/doc/howto/images/ydlidar_lidar_monitor.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:70856)
- [workspace/src/YDLidar-SDK/doc/images/EAI.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:74598)
- [workspace/src/YDLidar-SDK/doc/images/FlowChart.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:84629)
- [workspace/src/YDLidar-SDK/doc/images/YDLidar.jpg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:85509)
- [workspace/src/YDLidar-SDK/doc/images/YDLidar.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:86005)
- [workspace/src/YDLidar-SDK/doc/images/angle_q2.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:86043)
- [workspace/src/YDLidar-SDK/doc/images/communication_mechanism.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:86358)
- [workspace/src/YDLidar-SDK/doc/images/frame.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:88119)
- [workspace/src/YDLidar-SDK/doc/images/frame_intensity.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:89558)
- [workspace/src/YDLidar-SDK/doc/images/sdk_architecture.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:91013)
- [workspace/src/YDLidar-SDK/doc/images/sdk_init.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:91389)
- [workspace/src/YDLidar-SDK/doc/images/sdk_scanning.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:91736)
- [workspace/src/YDLidar-SDK/doc/images/sequence.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:92909)
- [workspace/src/YDLidar-SDK/doc/images/system_workflow.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:93640)
- [workspace/src/YDLidar-SDK/doc/quickstart/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:96655)
- [workspace/src/YDLidar-SDK/doc/quickstart/ydlidar_sdk_software_installation_guide.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:96673)
- [workspace/src/YDLidar-SDK/doc/tutorials/examine_the_simple_lidar_tutorial.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:96715)
- [workspace/src/YDLidar-SDK/doc/tutorials/writing_lidar_network_adapter_tutorial_c++.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:96785)
- [workspace/src/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_c++.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:97156)
- [workspace/src/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_c.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:97531)
- [workspace/src/YDLidar-SDK/doc/tutorials/writing_lidar_tutorial_python.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:97895)
- [workspace/src/YDLidar-SDK/examples/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98051)
- [workspace/src/YDLidar-SDK/examples/data/lidar0.csv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98097)
- [workspace/src/YDLidar-SDK/examples/data/lidar1.csv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98264)
- [workspace/src/YDLidar-SDK/examples/data/lidar2.csv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98431)
- [workspace/src/YDLidar-SDK/examples/et_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98598)
- [workspace/src/YDLidar-SDK/examples/gs_ota.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:98799)
- [workspace/src/YDLidar-SDK/examples/gs_restart.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:99056)
- [workspace/src/YDLidar-SDK/examples/gs_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:99325)
- [workspace/src/YDLidar-SDK/examples/gs_test2.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:99641)
- [workspace/src/YDLidar-SDK/examples/lidar_c_api_test.c](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:100095)
- [workspace/src/YDLidar-SDK/examples/scl_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:100218)
- [workspace/src/YDLidar-SDK/examples/sdm18_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:100591)
- [workspace/src/YDLidar-SDK/examples/sdm_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:100831)
- [workspace/src/YDLidar-SDK/examples/tea_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:101052)
- [workspace/src/YDLidar-SDK/examples/tia_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:101311)
- [workspace/src/YDLidar-SDK/examples/tmini-sh_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:101446)
- [workspace/src/YDLidar-SDK/examples/tmini_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:101644)
- [workspace/src/YDLidar-SDK/examples/tof_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:101842)
- [workspace/src/YDLidar-SDK/examples/tri_and_gs_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102161)
- [workspace/src/YDLidar-SDK/examples/tri_restart.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102403)
- [workspace/src/YDLidar-SDK/examples/tri_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102551)
- [workspace/src/YDLidar-SDK/python/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102914)
- [workspace/src/YDLidar-SDK/python/examples/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102975)
- [workspace/src/YDLidar-SDK/python/examples/etlidar_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:102992)
- [workspace/src/YDLidar-SDK/python/examples/gs_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103027)
- [workspace/src/YDLidar-SDK/python/examples/plot_tof_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103072)
- [workspace/src/YDLidar-SDK/python/examples/plot_ydlidar_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103138)
- [workspace/src/YDLidar-SDK/python/examples/tea_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103200)
- [workspace/src/YDLidar-SDK/python/examples/test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103241)
- [workspace/src/YDLidar-SDK/python/examples/tminiplus_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103280)
- [workspace/src/YDLidar-SDK/python/examples/tof_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103324)
- [workspace/src/YDLidar-SDK/python/examples/tri_test.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103363)
- [workspace/src/YDLidar-SDK/python/numpy.i](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:103407)
- [workspace/src/YDLidar-SDK/python/test/pytest.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:106591)
- [workspace/src/YDLidar-SDK/python/ydlidar_sdk.i](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:106636)
- [workspace/src/YDLidar-SDK/setup.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:106729)
- [workspace/src/YDLidar-SDK/src/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:106817)
- [workspace/src/YDLidar-SDK/src/CYdLidar.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:106836)
- [workspace/src/YDLidar-SDK/src/CYdLidar.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:108629)
- [workspace/src/YDLidar-SDK/src/DTSLidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:109041)
- [workspace/src/YDLidar-SDK/src/DTSLidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:110042)
- [workspace/src/YDLidar-SDK/src/ETLidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:110256)
- [workspace/src/YDLidar-SDK/src/ETLidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:111540)
- [workspace/src/YDLidar-SDK/src/GSLidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:112000)
- [workspace/src/YDLidar-SDK/src/GSLidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:114041)
- [workspace/src/YDLidar-SDK/src/SDMLidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:114561)
- [workspace/src/YDLidar-SDK/src/SDMLidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:115566)
- [workspace/src/YDLidar-SDK/src/TiaLidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:116019)
- [workspace/src/YDLidar-SDK/src/TiaLidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:116979)
- [workspace/src/YDLidar-SDK/src/YDlidarDriver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:117240)
- [workspace/src/YDLidar-SDK/src/YDlidarDriver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:118907)
- [workspace/src/YDLidar-SDK/src/filters/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:119399)
- [workspace/src/YDLidar-SDK/src/filters/FilterInterface.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:119409)
- [workspace/src/YDLidar-SDK/src/filters/NoiseFilter.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:119450)
- [workspace/src/YDLidar-SDK/src/filters/NoiseFilter.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:120052)
- [workspace/src/YDLidar-SDK/src/filters/StrongLightFilter.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:120159)
- [workspace/src/YDLidar-SDK/src/filters/StrongLightFilter.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:120453)
- [workspace/src/YDLidar-SDK/src/ydlidar_sdk.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:120530)
- [workspace/src/YDLidar-SDK/src/ydlidar_sdk.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:120768)
- [workspace/src/YDLidar-SDK/startup/initenv.sh](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121039)
- [workspace/src/YDLidar-SDK/test/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121056)
- [workspace/src/YDLidar-SDK/test/lidar_test.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121072)
- [workspace/src/YDLidar-SDK/test/lidar_test.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121156)
- [workspace/src/YDLidar-SDK/ydlidar_config.h.in](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121195)

### workspace/src/custom_start

- [workspace/src/custom_start/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121229)
- [workspace/src/custom_start/README_report.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:121232)
- [workspace/src/custom_start/models/plane_test/materials/textures/circuit.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:122569)

### workspace/src/limo_ros2

- [workspace/src/limo_ros2/limo_car/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:142914)
- [workspace/src/limo_ros2/limo_car/README.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:142956)
- [workspace/src/limo_ros2/limo_car/Readme.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:143123)
- [workspace/src/limo_ros2/limo_car/meshes/limo_base.dae](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144172) — copia identica di `workspace/src/limo_ros2/limo_description/meshes/limo_base.dae`
- [workspace/src/limo_ros2/limo_car/meshes/limo_base.stl](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144176) — copia identica di `workspace/src/limo_ros2/limo_description/meshes/limo_base.stl`
- [workspace/src/limo_ros2/limo_car/meshes/limo_wheel.dae](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144180) — copia identica di `workspace/src/limo_ros2/limo_description/meshes/limo_wheel.dae`
- [workspace/src/limo_ros2/limo_car/meshes/limo_wheel.stl](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144184) — copia identica di `workspace/src/limo_ros2/limo_description/meshes/limo_wheel.stl`
- [workspace/src/limo_ros2/limo_car/rviz/gazebo.rviz](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144225)
- [workspace/src/limo_ros2/limo_car/rviz/urdf.rviz](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:144415)

### workspace/src/ros2_astra_camera

- [workspace/src/ros2_astra_camera/.clang-format](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:145163)
- [workspace/src/ros2_astra_camera/.gitattributes](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:145336)
- [workspace/src/ros2_astra_camera/.gitignore](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:145343)
- [workspace/src/ros2_astra_camera/README.MD](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:146114)
- [workspace/src/ros2_astra_camera/astra_camera/.clang-format](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:146543)
- [workspace/src/ros2_astra_camera/astra_camera/.clang-tidy](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:146716)
- [workspace/src/ros2_astra_camera/astra_camera/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:146870)
- [workspace/src/ros2_astra_camera/astra_camera/config/.gitkeep](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:147147)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/constants.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:147150)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/dynamic_params.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:147278)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/json.hpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:147334)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/ob_camera_node.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:171676)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/ob_camera_node_factory.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:171957)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/ob_context.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172037)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/ob_timer_filter.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172098)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/point_cloud_proc/depth_traits.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172145)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/point_cloud_proc/point_cloud_proc.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172220)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/point_cloud_proc/point_cloud_xyz.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172230)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/point_cloud_proc/point_cloud_xyzrgb.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172317)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/ros_param_backend.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172424)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/types.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172459)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/utils.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172530)
- [workspace/src/ros2_astra_camera/astra_camera/include/astra_camera/uvc_camera_driver.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172608)
- [workspace/src/ros2_astra_camera/astra_camera/include/magic_enum/magic_enum.hpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:172775)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/Android-Arm/OniPlatformAndroid-Arm.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174319)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/Driver/OniDriverAPI.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174368)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/Driver/OniDriverTypes.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174781)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/KinectProperties.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174841)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/Linux-Arm/OniPlatformLinux-Arm.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174951)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/Linux-x86/OniPlatformLinux-x86.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:174993)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/MacOSX/OniPlatformMacOSX.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175101)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniCAPI.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175164)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniCEnums.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175453)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniCProperties.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175571)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniCTypes.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175657)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniEnums.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:175912)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniPlatform.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:176039)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniProperties.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:176118)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniTest.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:176207)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OniVersion.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:176253)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/OpenNI.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:176308)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/PS1080.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:179125)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/PSLink.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:180340)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/PrimeSense.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:180585)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/SonixCamera.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:180851)
- [workspace/src/ros2_astra_camera/astra_camera/include/openni2/util.h](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181256)
- [workspace/src/ros2_astra_camera/astra_camera/launch/.gitkeep](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181337)
- [workspace/src/ros2_astra_camera/astra_camera/launch/astra.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181340)
- [workspace/src/ros2_astra_camera/astra_camera/launch/astra_pro.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181438)
- [workspace/src/ros2_astra_camera/astra_camera/launch/astro_pro_plus.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181566)
- [workspace/src/ros2_astra_camera/astra_camera/launch/dabai_dc1.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181694)
- [workspace/src/ros2_astra_camera/astra_camera/launch/dabai_dcw.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181822)
- [workspace/src/ros2_astra_camera/astra_camera/launch/dabai_dw.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:181949)
- [workspace/src/ros2_astra_camera/astra_camera/launch/dabai_pro.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182019)
- [workspace/src/ros2_astra_camera/astra_camera/launch/dabai_u3.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182092)
- [workspace/src/ros2_astra_camera/astra_camera/launch/deeyea.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182220)
- [workspace/src/ros2_astra_camera/astra_camera/launch/embedded_s.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182348)
- [workspace/src/ros2_astra_camera/astra_camera/launch/embedded_u3.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182476)
- [workspace/src/ros2_astra_camera/astra_camera/launch/gemini.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182604)
- [workspace/src/ros2_astra_camera/astra_camera/launch/gemini_e.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182732)
- [workspace/src/ros2_astra_camera/astra_camera/launch/gemini_e_lite.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182860)
- [workspace/src/ros2_astra_camera/astra_camera/launch/list_devices.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182962)
- [workspace/src/ros2_astra_camera/astra_camera/launch/multi_astra.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:182972)
- [workspace/src/ros2_astra_camera/astra_camera/launch/multi_dabai_dcw.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183004)
- [workspace/src/ros2_astra_camera/astra_camera/launch/multi_dabai_pro.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183050)
- [workspace/src/ros2_astra_camera/astra_camera/launch/multi_deeyea.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183081)
- [workspace/src/ros2_astra_camera/astra_camera/launch/multi_gemini.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183114)
- [workspace/src/ros2_astra_camera/astra_camera/launch/stereo_s.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183146)
- [workspace/src/ros2_astra_camera/astra_camera/launch/stereo_s_u3.launch.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183272)
- [workspace/src/ros2_astra_camera/astra_camera/package.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183400)
- [workspace/src/ros2_astra_camera/astra_camera/rviz/.gitkeep](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183442)
- [workspace/src/ros2_astra_camera/astra_camera/rviz/multi_camera.rviz](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183445)
- [workspace/src/ros2_astra_camera/astra_camera/rviz/pointcloud.rviz](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183722)
- [workspace/src/ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:183944)
- [workspace/src/ros2_astra_camera/astra_camera/scripts/depth_to_color.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184025)
- [workspace/src/ros2_astra_camera/astra_camera/scripts/format_output_camera_params.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184090)
- [workspace/src/ros2_astra_camera/astra_camera/scripts/format_output_supported_video_modes.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184125)
- [workspace/src/ros2_astra_camera/astra_camera/scripts/install.sh](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184167)
- [workspace/src/ros2_astra_camera/astra_camera/src/clean_up_shm_node.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184193)
- [workspace/src/ros2_astra_camera/astra_camera/src/dynamic_params.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184226)
- [workspace/src/ros2_astra_camera/astra_camera/src/list_devices_node.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184384)
- [workspace/src/ros2_astra_camera/astra_camera/src/main.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184429)
- [workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_info.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184458)
- [workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_node.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:184759)
- [workspace/src/ros2_astra_camera/astra_camera/src/ob_camera_node_factory.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:185356)
- [workspace/src/ros2_astra_camera/astra_camera/src/ob_context.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:185588)
- [workspace/src/ros2_astra_camera/astra_camera/src/ob_timer_filter.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:185671)
- [workspace/src/ros2_astra_camera/astra_camera/src/point_cloud_proc/point_cloud_xyz.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:185727)
- [workspace/src/ros2_astra_camera/astra_camera/src/point_cloud_proc/point_cloud_xyzrgb.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:185885)
- [workspace/src/ros2_astra_camera/astra_camera/src/ros_param_backend.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:186153)
- [workspace/src/ros2_astra_camera/astra_camera/src/ros_service.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:186190)
- [workspace/src/ros2_astra_camera/astra_camera/src/utils.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:186691)
- [workspace/src/ros2_astra_camera/astra_camera/src/uvc_camera_driver.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:186865)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187508)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/msg/DeviceInfo.msg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187556)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/msg/Extrinsics.msg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187570)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/msg/Metadata.msg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187579)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/package.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187587)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/GetCameraInfo.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187616)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/GetCameraParams.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187626)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/GetDeviceInfo.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187641)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/GetInt32.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187651)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/GetString.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187661)
- [workspace/src/ros2_astra_camera/astra_camera_msgs/srv/SetInt32.srv](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187671)

### workspace/src/ydlidar_ros2_driver

- [workspace/src/ydlidar_ros2_driver/CMakeLists.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187682)
- [workspace/src/ydlidar_ros2_driver/LICENSE.txt](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187790)
- [workspace/src/ydlidar_ros2_driver/README.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187793)
- [workspace/src/ydlidar_ros2_driver/config/ydlidar.rviz](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:187994)
- [workspace/src/ydlidar_ros2_driver/details.md](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:188177)
- [workspace/src/ydlidar_ros2_driver/images/EAI.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:188316)
- [workspace/src/ydlidar_ros2_driver/images/YDLidar.jpg](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:198347)
- [workspace/src/ydlidar_ros2_driver/images/cmake_error.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:198843)
- [workspace/src/ydlidar_ros2_driver/images/finished.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:200367)
- [workspace/src/ydlidar_ros2_driver/images/rviz.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:200729)
- [workspace/src/ydlidar_ros2_driver/images/view.png](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:201509)
- [workspace/src/ydlidar_ros2_driver/launch/ydlidar.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:203997)
- [workspace/src/ydlidar_ros2_driver/launch/ydlidar_launch.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204032)
- [workspace/src/ydlidar_ros2_driver/launch/ydlidar_launch_view.py](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204094)
- [workspace/src/ydlidar_ros2_driver/package.xml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204163)
- [workspace/src/ydlidar_ros2_driver/params/G1.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204202)
- [workspace/src/ydlidar_ros2_driver/params/G2.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204231)
- [workspace/src/ydlidar_ros2_driver/params/G6.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204261)
- [workspace/src/ydlidar_ros2_driver/params/GS2.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204290)
- [workspace/src/ydlidar_ros2_driver/params/TEA.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204325)
- [workspace/src/ydlidar_ros2_driver/params/TG.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204354)
- [workspace/src/ydlidar_ros2_driver/params/TminiPro.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204383)
- [workspace/src/ydlidar_ros2_driver/params/X2.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204413)
- [workspace/src/ydlidar_ros2_driver/params/X4-Pro.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204444)
- [workspace/src/ydlidar_ros2_driver/params/X4.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204473)
- [workspace/src/ydlidar_ros2_driver/params/ydlidar.yaml](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204502)
- [workspace/src/ydlidar_ros2_driver/src/scan_ranges.hpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204533)
- [workspace/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_client.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204567)
- [workspace/src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204615)
- [workspace/src/ydlidar_ros2_driver/startup/initenv.sh](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204891)
- [workspace/src/ydlidar_ros2_driver/test/test_scan_ranges.cpp](/home/giulio/Limo/DIFF_FOXY_WORKSPACE.patch:204908)

