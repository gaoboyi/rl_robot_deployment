/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/27/20.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <legged_hw/LeggedHWLoop.h>
#include "ti5HW.h"

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "legged_ti5_hw");
  // 创建ROS节点句柄
  ros::NodeHandle nh;
  // 创建用于访问私有命名空间参数的节点句柄
  ros::NodeHandle robot_hw_nh("~");

  // 创建并启动一个异步旋转器，允许多线程处理回调
  ros::AsyncSpinner spinner(4); // 使用4个线程
  spinner.start();

  try
  {
    // 创建硬件接口的共享指针实例
    std::shared_ptr<legged::Ti5HW> legged_ti5_hw = std::make_shared<legged::Ti5HW>();

    // 初始化硬件接口
    legged_ti5_hw->init(nh, robot_hw_nh);
    // 通过调用LeggedHWLoop类的构造函数，我们创建了一个名为control_loop的LeggedHWLoop对象，这个对象会启动硬件控制循环
    legged::LeggedHWLoop control_loop(nh, legged_ti5_hw);
    // 等待ROS节点关闭
    ros::waitForShutdown();
  }
  catch (const ros::Exception& e)
  {
    // 如果发生异常，打印错误信息并退出
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1; // 返回1表示异常退出
  }

  return 0; // 正常退出
}

