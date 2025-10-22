# ROS distribution to use
ARG ROS_DISTRO=jazzy

#################################
# ROS Base Image for precaching #
#################################
FROM osrf/ros:${ROS_DISTRO}-desktop AS base
##############
# Base Image #
##############
FROM srealper/roscon25-stonefish:stonefish_base AS stonefish_base

#####################
# Development Image #
#####################
FROM srealper/roscon25-stonefish:dev AS dev
