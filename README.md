# Malicious Collaborator Detection in Cooperative Mobility Tracking

## Introduction
This is the undergraduate thesis research at Peking University. The issue I deal with in this research is how to detect malicious collaborators (vehicles) that deliberately send bogus mobility observations to degrade the precision, stability and reliability of cooperative mobility tracking.

## Background
Recently, many cooperative mobility tracking algorithms are proposed to enhance the precision and reliability of mobility tracking when self-observation of a vehicle can't satisfy the precision and reliability required in Intelligent Transportation Systems (ITS) and Autonomous Driving (AD). The main idea of most algorithm is let vehicles share their observations of other nearby vehicles with others and then use some fusion algorithm to enhance the mobility tracking.

However, those algorithms doesn't consider possible malicious collaborators (nearby vehicles) which can deliberately share some bogus observations to degrade the precision and reliability of mobility tracking of other vehicles. So I proposed two sequential clustering algorithms: Dynamic Model based Mean State Detection (DMMSD) and Mean Residue Error Detection (MRED) to tackle this issue. For the detail of background, related work and proposed algorithms, please refer to my paper. 

## Codes Organization Hierarchy
Organization hierarchy of all .m files is shown below:

Test_Bench.m <br>
--> CoopTracking_MalDetection.m <br>
&emsp;&emsp;--> SeqDetector.m <br>
&emsp;&emsp;&emsp;&emsp;--> SeqMMSE.m <br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;-->SeqMMSE_search.m <br>
&emsp;&emsp;&emsp;&emsp;--> DMMSD.m <br>
&emsp;&emsp;&emsp;&emsp;--> MRED.m <br>
&emsp;&emsp;&emsp;&emsp;-->Robust_ML.m <br>
&emsp;&emsp;--> LMS.m <br>
&emsp;&emsp;--> MAE.m <br>

Test_Bench.m  is the main entrance where you can set all parameters of simulation. (e.g. variance of observations, number of cooperative vehicles, number of malicious vehicles, test mode and so on.)

Test_Bench.m will only call CoopTracking_MalDetection.m where cooperative mobility tracking and malicious collaborator detection are implemented. All filtering and detection algorithms (LMS.m, MAE.m, SeqMMSE.m, DMMSD.m, MRED.m) will be called from here. Filtering algorithms can be directly called in CoopTracking_MalDetection.m, while to call detection algorithm, firstly SeqDetector.m is called then detection algorithms are called inside SeqDetector.m.

All detection algorithms implemented here are sequential algorithms, which means they need to collect data for a period before they can start working. To tackle this cold start issue, a simple robust maximum likelihood filtering is used in SeqDetector.m before sequential detection algorithms are ready.  