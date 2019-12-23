# Malicious Collaborator Detection in Cooperative Mobility Tracking

## Introduction
This is the undergraduate thesis research at Peking University. The issue I deal with in this research is how to detect malicious collaborators (vehicles) that deliberately send bogus mobility observations to degrade the precision, stability and reliability of cooperative mobility tracking.

## Background
Recently, many cooperative mobility tracking algorithms are proposed to enhance the precision and reliability of mobility tracking when self-observation of a vehicle can't satisfy the precision and reliability required in Intelligent Transportation Systems (ITS) and Autonomous Driving (AD). The main idea of most algorithm is let vehicles share their observations of other nearby vehicles with others and then use some fusion algorithm to enhance the mobility tracking.

However, those algorithms doesn't consider possible malicious collaborators (nearby vehicles) which can deliberately share some bogus observations to degrade the precision and reliability of mobility tracking of other vehicles. So I proposed two sequential clustering algorithms: Dynamic Model based Mean State Detection (DMMSD) and Mean Residue Error Detection (MRED) to tackle this issue. For the detail of background, related work and proposed algorithms, please refer to my paper. 

## Code Organization Hierarchy
Test_Bench.m
-->KF_multivehicles.m


Test_Bench.m 