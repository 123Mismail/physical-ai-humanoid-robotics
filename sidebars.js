/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main textbook sidebar
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'c1-foundations-physical-ai',
        'c2-ros2-architecture',
        'c3-ros2-actions',
        'c4-urdf-robot-description',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      items: [
        'c5-gazebo-simulation',
        'c6-isaac-sim',
        'c7-unity-simulation',
        'c8-advanced-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Edge Computing and Embedded Systems',
      items: [
        'c9-real-time-control',
        'c10-real-time-algorithms',
        'c11-sensor-fusion',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Humanoid Integration',
      items: ['c12-whole-body-control', 'c13-zmp-walking', 'c14-humanoid-integration'],
    },
  ],
};

module.exports = sidebars;
