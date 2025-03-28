/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_REACHABILITYEVALUATIONSTATE_H
#define YARP_THRIFT_GENERATOR_STRUCT_REACHABILITYEVALUATIONSTATE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class ReachabilityEvaluationState :
        public yarp::os::idl::WirePortable
{
public:
    // Fields
    std::string status{};
    yarp::sig::Matrix reached_pose{};

    // Default constructor
    ReachabilityEvaluationState() = default;

    // Constructor with field values
    ReachabilityEvaluationState(const std::string& status,
                                const yarp::sig::Matrix& reached_pose);

    // Read structure on a Wire
    bool read(yarp::os::idl::WireReader& reader) override;

    // Read structure on a Connection
    bool read(yarp::os::ConnectionReader& connection) override;

    // Write structure on a Wire
    bool write(const yarp::os::idl::WireWriter& writer) const override;

    // Write structure on a Connection
    bool write(yarp::os::ConnectionWriter& connection) const override;

    // Convert to a printable string
    std::string toString() const;

    // If you want to serialize this class without nesting, use this helper
    typedef yarp::os::idl::Unwrapped<ReachabilityEvaluationState> unwrapped;

private:
    // read/write status field
    bool read_status(yarp::os::idl::WireReader& reader);
    bool write_status(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_status(yarp::os::idl::WireReader& reader);
    bool nested_write_status(const yarp::os::idl::WireWriter& writer) const;

    // read/write reached_pose field
    bool read_reached_pose(yarp::os::idl::WireReader& reader);
    bool write_reached_pose(const yarp::os::idl::WireWriter& writer) const;
    bool nested_read_reached_pose(yarp::os::idl::WireReader& reader);
    bool nested_write_reached_pose(const yarp::os::idl::WireWriter& writer) const;
};

#endif // YARP_THRIFT_GENERATOR_STRUCT_REACHABILITYEVALUATIONSTATE_H
