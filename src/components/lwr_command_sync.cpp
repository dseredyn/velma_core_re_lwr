/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <std_msgs/Int32.h>

using namespace RTT;

namespace velma_core_re_lwr_types {

class LwrCommandSync: public RTT::TaskContext {
public:
    explicit LwrCommandSync(const std::string &name);

    bool startHook();

    void updateHook();

private:
    typedef boost::array<double, 7 > Joints;

    // ports
    RTT::InputPort<Joints > port_t_in_;
    RTT::InputPort<std_msgs::Int32 > port_KRL_CMD_in_;

    RTT::OutputPort<Joints > port_t_out_;
    RTT::OutputPort<std_msgs::Int32 > port_KRL_CMD_out_;

    Joints t_;
    std_msgs::Int32 KRL_CMD_;

    bool valid_t_prev_;
    bool valid_t_prev_2_;
    bool valid_KRL_CMD_prev_;
    bool valid_KRL_CMD_prev_2_;
};

LwrCommandSync::LwrCommandSync(const std::string &name)
    : TaskContext(name)
    , valid_t_prev_(false)
    , valid_t_prev_2_(false)
    , valid_KRL_CMD_prev_(false)
    , valid_KRL_CMD_prev_2_(false)
{
    this->ports()->addPort("t_INPORT", port_t_in_);
    this->ports()->addPort("KRL_CMD_INPORT", port_KRL_CMD_in_);

    this->ports()->addPort("t_OUTPORT", port_t_out_);
    this->ports()->addPort("KRL_CMD_OUTPORT", port_KRL_CMD_out_);
}

bool LwrCommandSync::startHook() {
    return true;
}

void LwrCommandSync::updateHook() {
    bool valid_t = (port_t_in_.read(t_) == RTT::NewData);
    bool valid_KRL_CMD = (port_KRL_CMD_in_.read(KRL_CMD_) == RTT::NewData);

    if (valid_t || valid_t_prev_ || valid_t_prev_2_) {
        port_t_out_.write(t_);
    }

    if (valid_KRL_CMD || valid_KRL_CMD_prev_ || valid_KRL_CMD_prev_2_) {
        port_KRL_CMD_out_.write(KRL_CMD_);
    }

    valid_t_prev_2_ = valid_t_prev_;
    valid_t_prev_ = valid_t;
    valid_KRL_CMD_prev_2_ = valid_KRL_CMD_prev_;
    valid_KRL_CMD_prev_ = valid_KRL_CMD;
}

}   // velma_core_re_lwr_types

ORO_LIST_COMPONENT_TYPE(velma_core_re_lwr_types::LwrCommandSync)

