#include "dxGrasp.h"
#include "dxRobot.h"
#include "dxKin.h"
//#include "dxGrasp.h" NOT WORKING IF INCLUDED HERE - To investiguate
#include "dxTrajectory.h"
#include "dxTrajectoryRigidBody.h"
#include "dxTimer.h"
#include <corecrt_math_defines.h>
#include <visp3/core/vpPoseVector.h>
#include <map>
#include <future>

using namespace std;

typedef vector<double> VecD;
void printMsg(string message, string prep = "[Teleop] ", string end = "\n");
void waitForUserInput(string message = "Press ENTER");
void printVec(vpColVector v, string name, string end = "\n");

template<typename T>
void printVec(vector<T> v, string name, string end = "\n")
{
    cout << " " << name.c_str() << " ";
    for (int i = 0; i < v.size(); i++)
        cout << v[i] << " ";
    cout << end;
}
Eigen::Matrix4d getEigenMatFrmPose(VecD X);
vpHomogeneousMatrix getCompositionTransThenRot(vpHomogeneousMatrix H1, vpHomogeneousMatrix H2);
vpHomogeneousMatrix getVispHomogeneousFrmPose(VecD X);
VecD getPoseFrmVispHomogeneous(vpHomogeneousMatrix H);
vpColVector getQuaternionError(vpQuaternionVector p1, vpQuaternionVector p2);
vpColVector getPoseForceError(vpPoseVector p1, vpPoseVector p2);
vpRotationMatrix getRotationMatrix(double ux, double uy, double uz, double w);
VecD getVecFrmPose(vpPoseVector p);
vpPoseVector computePoseErrorInMaster(vpPoseVector pm, vpPoseVector ps, vpRotationMatrix RoMS);
vpPoseVector computeSlavePose(vpPoseVector pm, vpPoseVector pmi, vpPoseVector psi);
vpPoseVector computeMasterPose(vpPoseVector ps, vpPoseVector pmi, vpPoseVector psi);
vpColVector computeMasterForce(vpPoseVector pm, vpPoseVector ps, vpPoseVector pmi, vpPoseVector psi);
vpPoseVector getPoseVector(std::vector<double> p);
std::vector<double> getPoseFromHomogeneous(Eigen::Matrix4d H);
vpHomogeneousMatrix Eigne4dTovpHomMat(Eigen::Matrix4d M);
Eigen::Matrix4d vpHomMatToEigne4d(vpHomogeneousMatrix J);
VecD getTargetFrame(VecD X, Eigen::Matrix4d eeRt);
VecD getEEFrame(VecD X, Eigen::Matrix4d eeRt);
vector<int> queryKnearestIds(pcl::KdTreeFLANN<PointT> *kdtree, VecD Pose, int k);
bool kukaCheckIk(dxKukaIIWA *kuka, Kin *kin, VecD Xee, VecD q);

//Grasp Functions
bool getPointCloud(vision *vision, dxKukaIIWA *robot, Kin *kin, CloudPtr& cloud, CloudNormPtr& cloud_normals, double downSample);
bool loadPointCloud(vision *vision, CloudPtr& cloud, CloudNormPtr& cloud_normals, double downSample);
std::vector<dxGrasp::GraspInfo> getFeasibleGrasps(std::vector<dxGrasp::GraspInfo> grasps, dxGrasp  *grasp, Kin *kin, bool allAvailable = false);
std::vector<Grasp::GraspInfo> getGrasp(dxGrasp *grasp, Kin *kin, CloudPtr& cloud, CloudNormPtr& cloud_normals, CloudPtr& cloud_features, double downSample);

//Utils Functions
void startRobots(vector<RobotBase*> robots)
{
    for (auto it = robots.begin(); it != robots.end(); ++it)
    {
        cout << "[" << (*it)->getName() << "] Starting... " << endl;
        (*it)->start();
    }
}

void stopRobots(vector<RobotBase*> robots)
{
    for (auto it = robots.begin(); it != robots.end(); ++it)
    {
        cout << "[" << (*it)->getName() << "] Stopping... " << endl;
        (*it)->stop();
    }
}

void getLatency(vector<RobotBase*> robots)
{
    for (auto it = robots.begin(); it != robots.end(); ++it)
        cout << "[" << (*it)->getName() << "] Latency: " << (*it)->getLatency() << endl;
}

void main()
{
    //################################################
    //# Initialization (Robots, Kinematics, Grasp)
    //################################################
    printMsg("**GRASP TELEOPERATION DEMO**");

    //Acquired Point Cloud and Normals
    CloudPtr cloud(new Cloud);
    CloudPtr cloud_features(new Cloud);
    CloudNormPtr cloud_normals(new CloudNorm);

    //dxGrasp grasp;			//Grasp module
    //dxGrasp::GraspInfo gi;
    //gi.idxPt1 = 0;
    //gi.idxPt2 = 0;
    //PointT p;
    //cloud_features->push_back(p);
    //cloud_features->push_back(p);
    //cloud_features->push_back(p);
    //cloud_features->push_back(p);
    //grasp.detachView(cloud_features, gi);
    //waitForUserInput();
    //return;


    //dxVirtuose6D virtuose;	//Virtuose Robot
    //virtuose.start();
    //virtuose.makeAsync(STATE_CART, CTRL_FORCE_CART);

    //while (true)
    //{
    //    VecD qk = virtuose.getJntPos();
    //    VecD Xk = virtuose.getPosCarte();
    //    vpPoseVector Xp = getPoseVector(Xk);
    //    vpHomogeneousMatrix H(Xp);

    //    VecD h;
    //    H.convert(h);

    //    printVec<double>(h, "[H]", "\r");
    //    this_thread::sleep_for(chrono::milliseconds(1));
    //}
    //exit(0);

    //Declarations
    dxKukaIIWA kuka;		//Kuka Robot
    dxVirtuose6D virtuose;	//Virtuose Robot
    dxGrasp grasp;			//Grasp module
    Kin kin;				//Kinematic Module


    //dxTrajectory traj;		//Trajectory Objects
    dxTrajectoryRB trajrb;	//Rigid Body Trajectory
    dxTrajectoryRB trajGrasp;	//Rigid Body Trajectory for Grasp
    dxTrajectoryRB trajPostGrasp;	//Rigid Body Trajectory for postGrasp
    dxTimer timer;

    //Target Frame in End effector (In order to align the frame of the kuka to th one of the virtuose)
    Eigen::Matrix4d eeRt;
    eeRt << 0, 0, -1, 0,
         0, 1, 0, 0,
         1, 0, 0, 0,
         0, 0, 0, 1;

    //Custom Initialization of Schunk Gripper
    dxSchunkPG70::Params schunkParams;
    schunkParams.port = "COM3";
    dxSchunkPG70 schunk(schunkParams);

    //Various local robot state variables
    dxVirtuose6D::State virtState(dxVirtuose6D::nbJoints);
    dxKukaIIWA::State kukaState(dxKukaIIWA::nbJoints);
    dxSchunkPG70::State schunkState(dxSchunkPG70::nbJoints);

    //List of Robots
    vector<RobotBase*> robots = { &kuka, &virtuose, &schunk };

    //Starting robots
    startRobots(robots);

    //Get the latency for INFO
    getLatency(robots);

    //Detach the Controllers from this thread
    //-Kuka Driver is already async => no need to detach it
    //-Schunk Can't work in async mode (Freeze)
    virtuose.makeAsync(STATE_CART, CTRL_FORCE_CART);

    //Opening the Schunk Gripper
    schunk.openGripper();

    //################################################
    //# Main Loop (Visualization, Grasp and Teleop)
    //################################################
    printMsg("**Launchin main DEMO**");

    //Declarations of the demo variables
    VecD masterXi, slaveXi, slaveQi, schunkQi;	//Master cartesian Pose, Slave cartesian pose, slave joint Pose, Schunk joint pose
    vpPoseVector slavePi, masterPi;				//Utility for VecD to VpPoseVector conversion
    vector<double> Ftm1(RobotBase::rbTDim);
    vector<double> dF(RobotBase::rbTDim);
    //vector<double> dFmax(RobotBase::rbTDim, 8);
    vector<double> dFmax(RobotBase::rbTDim, 15);
    bool ESTOP = false;
    int graspId = 0;
    bool isGraspsAvailable = false;
    bool isViewDetached = false;
    bool scan = true;
    vector<Button> buttonsVirtuose;
    std::vector<Grasp::GraspInfo> graspsInfo;
    pcl::KdTreeFLANN<PointT> kdtreeTraj;
    //dxTrajectory::SeqPointNT seqTraj;
    dxTrajectoryRB::SeqRigidBodyT seqTrajrb;
    dxGraspVisualizer::SceneData scene;
    int trajNPts = 1000;

    //Haptic Gains
    double fGainTeleop = 50.0;
    double tGainTeleop = 2.0;
    double fGainGuidedHaptic = 300.0;
    double tGainGuidedHaptic = 0.0;

    enum Mode
    {
        INITSTATE,
        RESET,
        SCANNING,
        TELEOPERATION,
        GUIDED_TRAJ,
        GUIDED_HAPTIC,
        AUTONOMOUS,
        DEFAULT
    };
    Mode mode = INITSTATE;
    Mode modePrevious = mode;

    bool blockGuidedMode = false;
    bool modeGuidedTrajCloseGripper = false;

    VecD slaveXsnap;
    //Run until ESTOP (... or TO IMPLEMENT)
    while(!ESTOP)
    {
        //Update the State
        VecD masterX = virtuose.getPosCarte();
        VecD slaveQ = kuka.getJntPos();
        kin.setJntPos(slaveQ);
        VecD slaveXee = kin.getfk7x1();
        VecD slaveX = getTargetFrame(slaveXee, eeRt);
        //VecD gripperQ = schunk.getJntPos();

        //Update Scene
        Eigen::Matrix4d slaveX4d = getEigenMatFrmPose(slaveXee);
        scene.m = slaveX4d * grasp.getRgripperInEE();
        scene.gripperJntPose = dxSchunkPG70::Params().q_max[0];	//Always Open => Problem with communication with Gripper. Doesn't work in continuous of getJntPos()

        //Get Virtuose Buttons
        buttonsVirtuose = virtuose.getButtons();

        //Handle the Sate Machine via Buttons
        if (buttonsVirtuose[VIRTUOSE_BTN_FF].get())
        {
            if (buttonsVirtuose[VIRTUOSE_BTN_POWER].get())
            {
                mode = TELEOPERATION;

                if (isGraspsAvailable)
                {
                    if (buttonsVirtuose[VIRTUOSE_BTN_RIGHT].get())
                    {
                        mode = GUIDED_TRAJ;
                    }

                    if (buttonsVirtuose[VIRTUOSE_BTN_LEFT].get())
                    {
                        mode = GUIDED_HAPTIC;
                    }
                }

                ////Force Initialization of state If Right/Left button Released, Middle Button Holded, or previous state = Default
                if(buttonsVirtuose[VIRTUOSE_BTN_RIGHT].isTriggeredUp()
                        || buttonsVirtuose[VIRTUOSE_BTN_LEFT].isTriggeredUp()
                        || buttonsVirtuose[VIRTUOSE_BTN_MIDDLE].get()
                        || (modePrevious == DEFAULT))
                {
                    mode = INITSTATE;
                }
            }
            else
            {
                mode = DEFAULT;

                if (buttonsVirtuose[VIRTUOSE_BTN_STOP].get())
                {
                    mode = SCANNING;
                }

                //Force Initialization of state
                if (modePrevious == SCANNING)
                {
                    mode = INITSTATE;
                }
            }
        }
        else
        {
            mode = DEFAULT;

            if (buttonsVirtuose[VIRTUOSE_BTN_BREAK].isTriggeredUp())
                mode = RESET;
        }


        switch (mode)
        {
        case INITSTATE:
        {
            if (modePrevious != INITSTATE)
                printMsg("Mode: INITSTATE");

            //Get Initial Positon from the robots
            masterXi = virtuose.getPosCarte();
            slaveQi = kuka.getJntPos();
            //schunkQi = schunk.getJntPos();

            //Get Forward Kinematic of Kuka
            kin.setJntPos(slaveQi);
            VecD slaveXiee = kin.getfk7x1();
            slaveXi = getTargetFrame(slaveXiee, eeRt);

            //Convert VecD to VpPoseVector
            slavePi = getPoseVector(slaveXi);
            masterPi = getPoseVector(masterXi);

            virtuose.setForceCarte(VecD(RobotBase::rbTDim, 0.0));
            break;
        }
        case RESET:
        {
            if (modePrevious != RESET)
                printMsg("Mode: RESET");

            //Clear Errors and Open Gripper
            schunk.clearErrors();
            schunk.openGripper();

            //Send the robot to the zero Position
            VecD posDeg(7);
            posDeg[0] = 0;
            posDeg[1] = 0;
            posDeg[2] = 0;
            posDeg[3] = -90.0;
            posDeg[4] = 0;
            posDeg[5] = 90.0;
            posDeg[6] = 0;
            VecD posRad = kuka.toRad(posDeg);
            kuka.setJntPos(posRad);
            this_thread::sleep_for(chrono::seconds(5));

            //Set Zero as force for the Virtuose6D
            virtuose.setForceCarte(VecD(RobotBase::rbTDim, 0.0));

            break;
        }
        case SCANNING:
        {
            if (modePrevious != SCANNING)
                printMsg("Mode: SCANNING");

            isGraspsAvailable = false;

            //Reset the point cloud
            cloud.reset(new Cloud);
            cloud_normals.reset(new CloudNorm);
            cloud_features.reset(new Cloud);

            //double downSample = 0.004;
            double downSample = 0.004;

            //Acquire the point cloud
            context c;
            bool load = c.getLoadData();
            if (!load)
            {
                //Acquiring the point cloud
                if (!getPointCloud(grasp.getVisionPtr(), &kuka, &kin, cloud, cloud_normals, downSample))
                {
                    printMsg("Error Acquiring Point cloud");
                    return;
                }
            }
            else
            {
                if (!loadPointCloud(grasp.getVisionPtr(), cloud, cloud_normals, downSample))
                {
                    printMsg("Error Loading Point cloud");
                    return;
                }
            }

            //Geting Grasps
            graspsInfo = getGrasp(&grasp, &kin, cloud, cloud_normals, cloud_features, downSample);

            cout << "Found: " << graspsInfo.size() << " grasps" << endl;
            if (graspsInfo.size() == 0)
            {
                printMsg("Error No Grasps found");
                //return;
            }

            //Build the grasp tree
            if (graspsInfo.size())
            {
                isGraspsAvailable = true;
                grasp.set(graspsInfo);
                grasp.buildTree();

                //Visualization
                if (!isViewDetached)
                {
                    grasp.detachView(cloud_features, graspsInfo[0]);
                    isViewDetached = true;
                }
            }

            break;
        }
        case TELEOPERATION:
        {
            if (modePrevious != TELEOPERATION)
                printMsg("Mode: TELEOPERATION");

            //Update the Visualization with the closest grasp

            if (isGraspsAvailable)
            {
                double x = slaveX[0];
                double y = slaveX[1];
                double z = slaveX[2];
                int k = 20;
                std::vector<dxGrasp::QueryData> qGrasp = grasp.queryKnearest(x, y, z, k);
                graspId = qGrasp[0].id;
                grasp.setCurrent(cloud_features, graspsInfo[graspId]);
            }

            //Update View
            if(isViewDetached)
                grasp.getViewPtr()->setSceneData(scene);

            //Compute the controllers (Slave position and Master force)
            vpPoseVector masterP = getPoseVector(masterX);
            vpPoseVector slaveP = getPoseVector(slaveX);
            vpPoseVector slavePose = computeSlavePose(masterP, masterPi, slavePi);
            vpColVector virtForce = computeMasterForce(masterP, slaveP, masterPi, slavePi);

            vpTranslationVector t = slavePose.getTranslationVector();
            vpQuaternionVector q;
            slavePose.extract(q);

            //Assign the slave pose to the Kuka
            VecD slaveXdes(RobotBase::rbQDim);
            slaveXdes[0] = t[0];
            slaveXdes[1] = t[1];
            slaveXdes[2] = t[2];
            slaveXdes[3] = q.x();
            slaveXdes[4] = q.y();
            slaveXdes[5] = q.z();
            slaveXdes[6] = q.w();

            //Transform the target frame to end effector
            VecD slaveEEdes = getEEFrame(slaveXdes, eeRt);

            //Set the Kuka Position
            if (kukaCheckIk(&kuka, &kin, slaveEEdes, slaveQ))
                kuka.setPosCarte(slaveEEdes);
            else
                printMsg("failed", "[Teleop] Kuka getIk()", "\r");

            //Compute and set the Virtuose Force
            VecD F(RobotBase::rbTDim);
            double fGain = fGainTeleop;
            double tGain = tGainTeleop;
            VecD gain = { fGain, fGain, fGain, tGain, tGain, tGain };
            for (int i = 0; i < RobotBase::rbTDim; i++)
            {
                Ftm1[i] = F[i];
                F[i] = gain[i] * virtForce[i];
                dF[i] = F[i] - Ftm1[i];

                if (fabs(dF[i]) > dFmax[i])
                {
                    ESTOP = true;
                    printMsg("Computed Force Exeeded the threshold => Exiting()");
                    //break;
                }
            }

            //Only set the forces when no emergency stop
            if (!ESTOP)
                virtuose.setForceCarte(F);

            //printVec<double>(dF, "[dF]", "\r");

            break;
        }
        case GUIDED_TRAJ:
        {
            //Go and Grasp if Key is pressed
            double T = 10;
            double Thalf = T / 2;
            if (modePrevious != GUIDED_TRAJ)
            {
                blockGuidedMode = false;
                modeGuidedTrajCloseGripper = false;
                printMsg("Mode: GUIDED_TRAJ");
                printMsg("Building Trajectory");
                VecD pPre = getPoseFromHomogeneous(graspsInfo[graspId].posePregraspEE);
                VecD pGrasp = getPoseFromHomogeneous(graspsInfo[graspId].poseEE);
                VecD pPost = getPoseFromHomogeneous(graspsInfo[graspId].posePostgraspEE);

                printVec<double>(pPre, "[pPre]", "\n");
                printVec<double>(pGrasp, "[pGrasp]", "\n");
                printVec<double>(pPost, "[pPost]", "\n");

                //Complete Rigid Body Trajectory
                trajrb.reset();
                trajrb.add(slaveXee);
                trajrb.add(pPre);
                trajrb.add(pGrasp);
                trajrb.add(pPost);
                trajrb.add(slaveXee);

                //Grasp Trajectory
                trajGrasp.reset();
                trajGrasp.add(slaveXee);
                trajGrasp.add(pPre);
                trajGrasp.add(pGrasp);

                //Post Grasp Trajectory
                trajPostGrasp.reset();
                trajPostGrasp.add(pGrasp);
                trajPostGrasp.add(pPost);
                trajPostGrasp.add(slaveXee);

                //Create the Trajectory
                printMsg("Building...");
                trajrb.create(T);
                trajGrasp.create(Thalf);
                trajPostGrasp.create(Thalf);

                //Get the Sequence for the entire trajectory
                seqTrajrb = trajrb.getTrajSeq(trajNPts);

                //Creating a vector of Points
                CloudPtr cloudSeqTraj(new Cloud);
                for (int i = 0; i < seqTrajrb.get().size(); i++)
                {
                    dxTrajectoryRB::RigidBodyT rbt = seqTrajrb.getRigidBodyT(i);
                    PointT p;
                    p.x = rbt.get()[0];
                    p.y = rbt.get()[1];
                    p.z = rbt.get()[2];
                    p.r = 0;
                    p.g = 0;
                    p.b = 255;
                    p.a = 255 - static_cast<unsigned char>(255 * rbt.t / trajrb.getDuration());

                    cloudSeqTraj->points.push_back(p);
                }

                //Update Scene with trajectory
                scene.clearCloud();
                scene.addCloud(cloudSeqTraj);

                //Add the point at T=0
                CloudPtr cloudTrajCurrentPt(new Cloud);
                PointT p;
                p.x = seqTrajrb.getRigidBody(0).p.x;
                p.y = seqTrajrb.getRigidBody(0).p.y;
                p.z = seqTrajrb.getRigidBody(0).p.z;
                p.r = 0;
                p.g = 0;
                p.b = 255;
                cloudTrajCurrentPt->points.push_back(p);
                scene.addCloud(cloudTrajCurrentPt, 8);

                printMsg("Done");

                //Initialize the timer
                timer.tick();

            }

            if (!blockGuidedMode)
            {
                double t = timer.tockUpperLimit(T);
                VecD X;

                //Split the trajectory in two Grasp and PostGrasp
                if(t < Thalf)
                {
                    //Get next position on trajectory
                    X = trajGrasp.get(t);
                }
                else
                {
                    //Close the gripper once the grasp pose has been reached
                    if(!modeGuidedTrajCloseGripper)
                    {
                        schunk.closeGripper();
                        modeGuidedTrajCloseGripper = true;
                        this_thread::sleep_for(chrono::seconds(3));
                    }

                    //Get next position on trajectory
                    double _t = t - Thalf;
                    X = trajPostGrasp.get(_t);
                }

                //Set the Kuka Position
                kuka.setPosCarte(X);

                //Add the current Point Selected
                CloudPtr cloudTrajCurrentPt(new Cloud);
                PointT p;
                p.x = X[0];
                p.y = X[1];
                p.z = X[2];
                p.r = 255;
                p.g = 0;
                p.b = 0;
                cloudTrajCurrentPt->points.push_back(p);
                scene.insert(cloudTrajCurrentPt, 1, 12);

            }
            //printMsg("Timer: " + to_string(t) + "/" + to_string(T));

            break;
        }
        case GUIDED_HAPTIC:
        {
            //Go and Grasp if Key is pressed
            if (modePrevious != GUIDED_HAPTIC)
            {
                printMsg("Mode: GUIDED_HAPTIC");
                printMsg("Building Trajectory");
                VecD pPre = getPoseFromHomogeneous(graspsInfo[graspId].posePregraspEE);
                VecD pGrasp = getPoseFromHomogeneous(graspsInfo[graspId].poseEE);
                VecD pPost = getPoseFromHomogeneous(graspsInfo[graspId].posePostgraspEE);

                printVec<double>(slaveXee, "[slaveXee]", "\n");
                printVec<double>(masterX, "[masterX]", "\n");
                printVec<double>(pPre, "[pPre]", "\n");
                printVec<double>(pGrasp, "[pGrasp]", "\n");
                printVec<double>(pPost, "[pPost]", "\n");

                //Converting Pose Vectors to Visp
                //vpHomogeneousMatrix vpHslave = getVispHomogeneousFrmPose(slaveXee);
                //vpHomogeneousMatrix vpHpre = getVispHomogeneousFrmPose(pPre);
                //vpHomogeneousMatrix vpHgrasp = getVispHomogeneousFrmPose(pGrasp);
                //vpHomogeneousMatrix vpHmaster = getVispHomogeneousFrmPose(masterX);

                //Convert VecD to Visp Pose
                //VecD slaveXtarget = getEEFrame(slaveXee, eeRt);
                //VecD preGraspTarget = getEEFrame(pPre, eeRt);
                //VecD graspTarget = getEEFrame(pGrasp, eeRt);

                //vpPoseVector slavePose = getPoseVector(slaveXtarget);
                //vpPoseVector masterPose = getPoseVector(masterX);
                //vpPoseVector preGraspPose = getPoseVector(preGraspTarget);
                //vpPoseVector graspPose = getPoseVector(graspTarget);

                //vpPoseVector masterPreGraspPose = computeMasterPose(preGraspPose, masterPose, slavePose);
                //vpPoseVector masterGraspPose = computeMasterPose(graspPose, masterPose, slavePose);

                ////Convert Master Homogeneous vector to Pose vector
                ////VecD masterXdes = getPoseFrmVispHomogeneous(vpHmaster);
                //VecD masterXdes = masterX;
                ////Build the trajectory in the master frame
                //trajrb.reset();
                //trajrb.add(masterX);
                ////Compute projected pregrasp Pose in Master frame
                //masterXdes = getVecFrmPose(masterPreGraspPose);
                //trajrb.add(masterXdes);
                ////Compute projected grasp Pose in Master frame
                //masterXdes = getVecFrmPose(masterGraspPose);
                //trajrb.add(masterXdes);

                VecD masterXee = getEEFrame(masterX, eeRt);

                vpPoseVector slavePose = getPoseVector(slaveXee);
                vpPoseVector masterPose = getPoseVector(masterXee);
                vpPoseVector preGraspPose = getPoseVector(pPre);
                vpPoseVector graspPose = getPoseVector(pGrasp);

                vpPoseVector masterPreGraspPose = computeMasterPose(preGraspPose, masterPose, slavePose);
                vpPoseVector masterGraspPose = computeMasterPose(graspPose, masterPose, slavePose);

                //Convert Master Homogeneous vector to Pose vector
                //VecD masterXdes = getPoseFrmVispHomogeneous(vpHmaster);
                VecD masterXdes = masterX;
                //Build the trajectory in the master frame
                trajrb.reset();
                trajrb.add(masterX);
                //Compute projected pregrasp Pose in Master frame
                masterXdes = getVecFrmPose(masterPreGraspPose);
                trajrb.add(masterXdes);
                //Compute projected grasp Pose in Master frame
                masterXdes = getVecFrmPose(masterGraspPose);
                trajrb.add(masterXdes);


                ////Info
                //cout << "[SlaveXee]" << endl << vpHslave << endl;
                //cout << "[SlavePre]" << endl << vpHpre << endl;
                //cout << "[SlaveGrasp]" << endl << vpHgrasp << endl;
                //cout << "[SlavePre] Rslave" << endl << vpHslave.inverse()*vpHpre << endl;
                //cout << "[SlaveGrasp] Rslave" << endl << vpHslave.inverse()*vpHgrasp << endl;
                //cout << "[MasterX]" << endl << vpHmaster << endl;
                //cout << "[Hmpre]" << endl << Hmpre << endl;
                //cout << "[Hmgrasp]" << endl << Hmgrasp << endl;
                //cout << "[Hmpre] Rmaster" << endl << Hmgrasp.inverse()*Hmpre << endl;
                //cout << "[Hmgrasp] Rmaster" << endl << Hmgrasp.inverse()*Hmgrasp << endl;
                //cout << "[sHm]" << endl << sHm << endl;
                //cout << "[MasterX]" << endl << masterPose << endl;
                //cout << "[MasterPre]" << endl << masterPreGraspPose << endl;
                //cout << "[MasterPost]" << endl << masterGraspPose << endl;

                printMsg("Building...");

                //Create the trajectory
                trajrb.create(1); // Duration of 1s

                //Get the trajectory
                seqTrajrb = trajrb.getTrajSeq(trajNPts);

                //Creating a vector of Points
                CloudPtr cloudSeqTraj(new Cloud);
                for (int i = 0; i < seqTrajrb.get().size(); i++)
                {
                    dxTrajectoryRB::RigidBodyT rbt = seqTrajrb.getRigidBodyT(i);
                    PointT p;
                    p.x = rbt.get()[0];
                    p.y = rbt.get()[1];
                    p.z = rbt.get()[2];
                    p.r = 255;
                    p.g = 0;
                    p.b = 0;
                    p.a = 255 - static_cast<unsigned char>(255 * rbt.t / trajrb.getDuration());

                    cloudSeqTraj->points.push_back(p);
                }

                //Create the query Grasph
                kdtreeTraj.setInputCloud(cloudSeqTraj);

                //Update Scene with trajectory
                scene.clearCloud();
                scene.addCloud(cloudSeqTraj);

                printMsg("Done");

            }

            //Get the Physical Position of the Virtuose6D
            vpPoseVector masterPose = getPoseVector(masterX);
            VecD Xr = { masterX[0], masterX[1], masterX[2] };

            //Query the id of the closests trajectory point
            vector<int> ids = queryKnearestIds(&kdtreeTraj, Xr, 1);
            dxTrajectoryRB::RigidBodyT ptTraj = seqTrajrb.getRigidBodyT(ids[0]);  //Get the closest trajectory point to the pose of the robot
            VecD Xt = { ptTraj.get()[0], ptTraj.get()[1], ptTraj.get()[2] };

            //Get The Pose vector of the projected point on the trajectory
            vpTranslationVector trajT(ptTraj.get()[0], ptTraj.get()[1], ptTraj.get()[2]);
            vpQuaternionVector trajQ(ptTraj.get()[3], ptTraj.get()[4], ptTraj.get()[5], ptTraj.get()[6]);
            vpRotationMatrix trajR(trajQ);
            vpPoseVector XtPose(trajT, trajR);

            //Get forces
            vpColVector Xf = getPoseForceError(XtPose, masterPose);

            //Compute the trajectory Forces
            VecD F(RobotBase::rbTDim);
            double fGain = fGainGuidedHaptic;
            double tGain = tGainGuidedHaptic;
            VecD gain = { fGain, fGain, fGain, tGain, tGain, tGain };

            //Compute the controllers (Slave position and Master force)
            vpPoseVector masterP = getPoseVector(masterX);
            //Force the rotation to the Target (Autonomous Rotation Mouvements)
            //masterP[3] = XtPose[3];
            //masterP[4] = XtPose[4];
            //masterP[5] = XtPose[5];
            //masterP[6] = XtPose[6];

            vpPoseVector slaveP = getPoseVector(slaveX);
            vpPoseVector _slavePose = computeSlavePose(masterP, masterPi, slavePi);
            //vpColVector virtForceCoupling = computeMasterForce(masterP, slaveP, masterPi, slavePi);

            vpTranslationVector t = _slavePose.getTranslationVector();
            vpQuaternionVector _q;
            _slavePose.extract(_q);

            //Assign the slave pose to the Kuka
            VecD slaveXdes(RobotBase::rbQDim);
            slaveXdes[0] = t[0];
            slaveXdes[1] = t[1];
            slaveXdes[2] = t[2];
            slaveXdes[3] = _q.x();
            slaveXdes[4] = _q.y();
            slaveXdes[5] = _q.z();
            slaveXdes[6] = _q.w();

            //Transform the target frame to end effector
            VecD slaveEEdes = getEEFrame(slaveXdes, eeRt);

            //Set the Kuka Position
            kuka.setPosCarte(slaveEEdes);

            VecD virtForceTraj = { Xt[0] - Xr[0],  Xt[1] - Xr[1],  Xt[2] - Xr[2], 0, 0, 0 };
            //VecD virtForceTraj = { Xf[0],  Xf[1],  Xf[2], Xf[3], Xf[4], Xf[5] };
            //Apply the force to the master
            for (int i = 0; i < RobotBase::rbTDim; i++)
            {
                Ftm1[i] = F[i];
                F[i] = gain[i] * virtForceTraj[i];
                dF[i] = F[i] - Ftm1[i];

                if (fabs(dF[i]) > dFmax[i])
                {
                    ESTOP = true;
                    printMsg("Computed Force Exeeded the threshold => Exiting()");
                    //break;
                }
            }

            //Only set the forces when no emergency stop
            if (!ESTOP)
                virtuose.setForceCarte(F);

            printVec<int>(ids, "[Info]", "/" + to_string(trajNPts));
            printVec<double>(F, "|", "\r");

            break;
        }
        case AUTONOMOUS:
        {
            if (modePrevious != AUTONOMOUS)
                printMsg("Mode: AUTONOMOUS");

            //Select Grasp using the buttons on the virtuose
            if (buttonsVirtuose[VIRTUOSE_BTN_LEFT].isTriggeredDown())
            {
                if (graspId > 0)
                {
                    graspId--;
                    printMsg("Grasp: " + to_string(graspId));
                }
            }

            if (buttonsVirtuose[VIRTUOSE_BTN_RIGHT].isTriggeredDown())
            {
                if (graspId < graspsInfo.size())
                {
                    graspId++;
                    printMsg("Grasp: " + to_string(graspId));
                }
            }

            //Set the current Grasp
            grasp.setCurrent(cloud_features, graspsInfo[graspId]);

            //Go and Grasp if Key is pressed
            if (buttonsVirtuose[VIRTUOSE_BTN_DEADMAN].get() & buttonsVirtuose[VIRTUOSE_BTN_RIGHT].isTriggeredDown())
            {
                printMsg("Building Trajectory");
                VecD pPre = getPoseFromHomogeneous(graspsInfo[graspId].posePregraspEE);
                VecD pGrasp = getPoseFromHomogeneous(graspsInfo[graspId].poseEE);
                VecD pPost = getPoseFromHomogeneous(graspsInfo[graspId].posePostgraspEE);

                //Get intial robot pose
                VecD slaveQsnapshot = kuka.getJntPos();
                VecD gripperQsnapshot = schunk.getJntPos();

                ////Open the Gripper
                //schunk.openGripper();

                ////Go to the Pregrasp Position
                //kuka.setPosCarte(pPre);
                //std::this_thread::sleep_for(std::chrono::seconds(5));

                ////Go to the Grasp Position
                //kuka.setPosCarte(pGrasp);
                //std::this_thread::sleep_for(std::chrono::seconds(3));

                ////Close the gripper
                //schunk.closeGripper();
                //std::this_thread::sleep_for(std::chrono::seconds(3));

                ////PostGrasp
                //kuka.setPosCarte(pPost);
                //std::this_thread::sleep_for(std::chrono::seconds(5));

                ////Send the robot the the initial pose
                //kuka.setJntPos(slaveQsnapshot);
                //std::this_thread::sleep_for(std::chrono::seconds(5));

            }

            break;
        }
        default:
        {
            if (modePrevious != DEFAULT)
                printMsg("Mode: DEFAULT");

            virtuose.setForceCarte(VecD(RobotBase::rbTDim, 0.0));
        }
        }

        modePrevious = mode;

        //Sleep 1ms
        this_thread::sleep_for(chrono::milliseconds(1));

        //Update the view
        if (isViewDetached)
            grasp.getViewPtr()->setSceneData(scene);


        //Open/Close SchunkPG70 Gripper
        {
            if (buttonsVirtuose[VIRTUOSE_BTN_ZELEC_MIN].isTriggeredDown())
                schunk.openGripper();

            if (buttonsVirtuose[VIRTUOSE_BTN_ZELEC_PLUS].isTriggeredDown())
                schunk.closeGripper();

        }

        //Display
        //printVec<double>(slaveQ, "[slaveQ]");
        //printVec<double>(slaveX, "[slaveX]", "\r");
        //printVec<double>(slaveXdes, "[slaveXdes]", "\r");
        //printVec<double>(F, "[virtForce]", "\r");
        //printVec<bool>(virtExtraState.serialize(), "[Extra State]", "\r");
        //cout << schunkQi[0] << "\r";
        //printVec(masterX, "[masterX] ");
        //cout << "[Buttons] - Left = " << virtExtraState.getBtnLeft() << " Middle = " << virtExtraState.getBtnMiddle() << " Right = " << virtExtraState.getBtnRight()  << " FF = " << virtExtraState.getBtnFF() << "\r";
    }
    printMsg("\nClosing Demo...");

}

bool kukaCheckIk(dxKukaIIWA *kuka, Kin *kin, VecD Xee, VecD q)
{
    //Get the Homogeneous matrix of the target Slave position
    vpHomogeneousMatrix Hp = getVispHomogeneousFrmPose(Xee);
    VecD qd;
    Hp.convert(qd);
    VecD qres(7);

    //Get the robot IK
    bool isKukaIK = kin->getIk(qd, q, qres);

    //Check for limits extra safety
    if (isKukaIK)
    {
        dxKukaIIWA::Params p;
        isKukaIK = p.isPosValid(kuka->toRad(qres));
    }

    return isKukaIK;
}

vector<int> queryKnearestIds(pcl::KdTreeFLANN<PointT> *kdtree, VecD Pose, int k)
{
    vector<VecD> res;
    PointT p;
    p.x = Pose[0];
    p.y = Pose[1];
    p.z = Pose[2];

//Get the closest points
    std::vector<int> ids(k);
    std::vector<float> distances(k);
    if (!kdtree->nearestKSearch(p, k, ids, distances))
        printMsg("[queryKnearestIds] Error");

    return ids;
}

bool getPointCloud(vision *vision, dxKukaIIWA *robot, Kin *kin, CloudPtr& cloud, CloudNormPtr& cloud_normals, double downSample)
{
    context c;
    cloud.reset(new Cloud);
    cloud_normals.reset(new CloudNorm);
    std::vector<Eigen::Matrix4d> poses;
    std::vector<CloudPtr> clouds;
    std::vector<CloudNormPtr> normals;
    Ensenso::ensenso sensor{};
    std::vector<std::vector<double>> cropVal = c.getCropBox();
    Ensenso::cropvalues crop;
    crop.xmin = cropVal[0][0];
    crop.xmax = cropVal[0][1];
    crop.ymin = cropVal[1][0];
    crop.ymax = cropVal[1][1];
    crop.zmin = cropVal[2][0];
    crop.zmax = cropVal[2][1];
    bool save = c.getSaveData();

    cout << "[Ensenso check] Checking camera connection..." << endl;
    if (!sensor.openDevice(0))
    {
        cout << "[ERROR] Opening the ensenso sensor" << endl;
        return false;
    }

    //Load the scan Poses
    vector<pair<vector<double>, double>> scanPosesD = c.getRobotPosesStamped();
    vector<vector<double>> scanPoses;
    vector<double> scanDurations;;
    cout << endl << yellow << "[Scan Poses]" << reset << endl;
    cout << setw(10);
    for (int j = 0; j < scanPosesD.size(); j++)
    {
        vector<double> _pose = scanPosesD[j].first;
        double duration = scanPosesD[j].second;

        //Convert Pose to radians

        scanPoses.push_back(_pose);
        scanDurations.push_back(duration);
        for (int k = 0; k < _pose.size(); k++)
        {
            cout << _pose[k] << setw(10);
        }
        cout << "| " << duration << "s" << endl;
    }
    cout << endl;

    //Send the robot to the scan poses
    string name = c.getDataset();
    string path = "data\\" + name + "\\";
    for (int i = 0; i < scanPoses.size(); i++)
    {
        std::cout << "[Scaning] # " << i + 1 << "/" << scanPoses.size() << std::endl;

        CloudPtr _cloud(new Cloud);
        _cloud.reset(new Cloud);
        CloudNormPtr _cloud_normals(new CloudNorm);
        _cloud_normals.reset(new CloudNorm);
        ofstream fcameraPose;

        //Set the joint position
        vector<double> sp = robot->toRad(scanPoses[i]);
        robot->setJntPos(sp);

        //cout << "Press ENTER to Continue" << endl;
        //getchar();
        std::this_thread::sleep_for(std::chrono::seconds(static_cast<long>(scanDurations[i])));

        //Capture the point cloud
        CloudMonoPtr _tempCloud(new CloudMono);
        sensor.capturePointCloud(*_tempCloud);
        pcl::copyPointCloud(*_tempCloud, *_cloud);

        //Save the images
        if (save)
        {
            sensor.saveRectifiedImages(path, "left", i);
            sensor.saveRectifiedImages(path, "right", i);
        }

        const Eigen::Matrix4f eMc = c.getHandEyeMatrix();

        //Get forward kinematics
        vector<double> X = kin->getfk7x1(robot->getJntPos());

        //Data Type Conversion
        vpTranslationVector T(X[0], X[1], X[2]);
        vpQuaternionVector Q(X[3], X[4], X[5], X[6]);
        vpHomogeneousMatrix J(T,Q);

        Eigen::Matrix4f bMe = dxGrasp::vpHomMatToEigne4f(J);

        //Compute Final Transformation for cloud to robot frame and it inverse
        Eigen::Matrix4f bMc = bMe * eMc;
        Eigen::Matrix4f cMb = bMc.transpose();

        //Save
        if (save)
        {
            string namefCameraPose = path + "camera_pose_" + std::to_string(i) + ".txt";
            fcameraPose.open(namefCameraPose);
            fcameraPose << bMc.matrix() << endl;
            fcameraPose.close();
        }
        //Tranform the point cloud to the base of the robot
        transformPointCloud(*_cloud, *_cloud, bMc, true);

        //Crop the Cloud
        sensor.cropbox(_cloud, crop, _cloud);

        if (_cloud->points.size() == 0)
        {
            std::cout << "[Scan] => No points... Next pose" << std::endl;
            continue;
        }

        //Saved the croped cloud projected in the base frame
        if (save)
        {
            string cloudNameBase = path + "cloud_rbase_" + std::to_string(i) + ".pcd";
            cout << "Saving:" << cloudNameBase << endl;
            io::savePCDFile(cloudNameBase.c_str(), *_cloud);

            CloudPtr _croppedCloudCamera(new Cloud);
            _croppedCloudCamera.reset(new Cloud);
            transformPointCloud(*_cloud, *_croppedCloudCamera, cMb, true);
            string cloudNameCamera = path + "cloud_rcamera_" + std::to_string(i) + ".pcd";
            io::savePCDFile(cloudNameCamera.c_str(), *_croppedCloudCamera);
        }

        //Cloud filtering
        vision->removeCloudOutliers(_cloud, _cloud);
        vision->downSample(_cloud, _cloud, downSample);

        //Compute the normals
        Eigen::Vector3f viewPoint;
        viewPoint(0) = bMc(0, 3);
        viewPoint(1) = bMc(1, 3);
        viewPoint(2) = bMc(2, 3);
        vision->getCloudNormals(_cloud, viewPoint, _cloud_normals);

        //Display the cloud with normals
        //vision->viewPointCloudWithNormals(_cloud, _cloud_normals);

        //Append the the clouds and normals list
        clouds.push_back(_cloud);
        normals.push_back(_cloud_normals);

        std::cout << "[Point Cloud] " << _cloud->points.size() << " Points" << std::endl;
    }

    //Register Views
    vision->stitchClouds(clouds, cloud);
    vision->stitchCloudsNormals(normals, cloud_normals);

    //Display the cloud with normals
    //vision->viewPointCloudWithNormals(cloud, cloud_normals);

    if (save)
    {
        string cloudNameRegistered = path + "cloud_registered.pcd";
        io::savePCDFile(cloudNameRegistered.c_str(), *cloud);
    }

    //Close the ensenso device
    sensor.closeDevice();
    return true;
}

bool loadPointCloud(vision *vision, CloudPtr& cloud, CloudNormPtr& cloud_normals, double downSample)
{
    cloud.reset(new Cloud);
    cloud_normals.reset(new CloudNorm);
    std::vector<Eigen::Matrix4d> poses;
    std::vector<CloudPtr> clouds;
    std::vector<CloudNormPtr> normals;
    context c;
    string dataset = c.getDataset();
    std::vector<std::vector<double>> cropVal = c.getCropBox();
    Ensenso::cropvalues crop;
    crop.xmin = cropVal[0][0];
    crop.xmax = cropVal[0][1];
    crop.ymin = cropVal[1][0];
    crop.ymax = cropVal[1][1];
    crop.zmin = cropVal[2][0];
    crop.zmax = cropVal[2][1];

    string path = "data\\" + dataset + "\\";
    for (int i = 0; i <= 4; i++)
    {
        CloudPtr _cloud(new Cloud);
        CloudNormPtr _cloud_normals(new CloudNorm);
        string nameCloud = path + "cloud_rbase_" + std::to_string(i) + ".pcd";
        string nameCameraPose = path + "camera_pose_" + std::to_string(i) + ".txt";

        std::cout << "[Loading] #" << i << " | " << nameCloud << " | " << nameCameraPose << std::endl;

        //Load the point cloud
        if (pcl::io::loadPCDFile<PointT>(nameCloud.c_str(), *_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file Cloud \n");
            return false;
        }


        //Load the camera poses
        ifstream _f(nameCameraPose);
        Eigen::Matrix4d _p;

        if (!_f.is_open())
        {
            std::cout << "[ERROR] Load poses => Invalid path" << std::endl;
            return false;
        }
        else
        {
            int ln = 0;
            string line;
            while (getline(_f, line))
            {
                istringstream iss(line);

                //Create a vector from the line
                vector<string> _vstr{ istream_iterator<string>{iss},
                                      istream_iterator<string>{} };

                if (_vstr.size() < 4)
                {
                    std::cout << "[ERROR] Load poses => Number of Columns Invalid" << std::endl;
                    return false;
                }
                else
                {
                    for (int j = 0; j < 4; j++)
                        _p(ln, j) = std::stod(_vstr[j]);

                    ln++;
                }
            }
            _f.close();
        }

        //vision->viewPointCloud(_cloud, "Object Scene", "");
        vision->removeCloudOutliers(_cloud, _cloud);
        //vision->viewPointCloud(_cloud, "Object Scene", "");
        vision->downSample(_cloud, _cloud, downSample);

        //Crop the Cloud
        ensenso::cropbox(_cloud, crop, _cloud);

        std::cout << "[Loading] Computing Normals" << std::endl;
        //Get the position of the viewpoint
        Eigen::Vector3f viewPoint;
        viewPoint(0) = _p(0, 3);
        viewPoint(1) = _p(1, 3);
        viewPoint(2) = _p(2, 3);
        vision->getCloudNormals(_cloud, viewPoint, _cloud_normals);
        //vision->viewPointCloudWithNormals(_cloud, _cloud_normals);

        poses.push_back(_p);
        clouds.push_back(_cloud);
        normals.push_back(_cloud_normals);
    }

    vision->stitchClouds(clouds, cloud);
    vision->stitchCloudsNormals(normals, cloud_normals);

    return true;
}

std::vector<dxGrasp::GraspInfo> getFeasibleGrasps(std::vector<dxGrasp::GraspInfo> grasps, dxGrasp  *grasp, Kin *kin, bool allAvailable)
{
//----------------MODIFICATION--------------------------
    std::vector<Grasp::GraspInfo> filteredGrasps;
    std::vector<Eigen::VectorXd> jntPoses;

//Look for poses which are reachable
    int numGrasps = 100;
    int maxEvaluatedGrasps = 1500;
    int pSize = min(maxEvaluatedGrasps, (int)grasps.size());

    if(allAvailable)
    {
        pSize = grasps.size();
        numGrasps = pSize;
    }

    std::cout << "Info" << endl;
    std::cout << "N Grasps Found: " << grasps.size() << endl;
    std::cout << "N Grasps used: " << pSize << endl;

    struct Data
    {
        bool valid;
        Grasp::GraspInfo grasp;

        Data()
        {
            valid = false;
        }

    };

//cout << "Initializing Async..." << endl;
    std::vector<future<Data>> futures;
    for (int i = 0; i < pSize; i++)
    {
        Grasp::GraspInfo g = grasps[i];
        Eigen::MatrixXd RgripperInEE = grasp->getRgripperInEE();

        ////Initializing the futures
        futures.push_back(std::async(std::launch::async, [](int sampleId, dxGrasp::GraspInfo _grasp, Kin *_kin, Eigen::Matrix4d Rge)
        {

            vector<double> qi(7);
            vector<double> qs(7);
            Data data;

            //Convert the grasp to the frame of the robot
            dxGrasp::PoseMap P = dxGrasp::poseToQuatGrasp(_grasp.pose, Rge);

            vector<double> Hvec;
            P.H.convert(Hvec);
            bool _res = _kin->getIk(Hvec, qi, qs);
            if (_res)
            {
                vector<double> qsPre(7);
                vector<double> qsPost(7);

                dxGrasp::PoseMap  Ppre = dxGrasp::poseToQuatGrasp(_grasp.posePregrasp, Rge);
                dxGrasp::PoseMap  Ppost = dxGrasp::poseToQuatGrasp(_grasp.posePostgrasp, Rge);

                Ppre.H.convert(Hvec);
                bool _resPre = _kin->getIk(Hvec, qs, qsPre);
                Ppost.H.convert(Hvec);
                bool _resPost = _kin->getIk(Hvec, qs, qsPost);

                if (_resPre && _resPost)
                {
                    Eigen::VectorXd jntPose(7);
                    jntPose[0] = qs[0];
                    jntPose[1] = qs[1];
                    jntPose[2] = qs[2];
                    jntPose[3] = qs[3];
                    jntPose[4] = qs[4];
                    jntPose[5] = qs[5];
                    jntPose[6] = qs[6];

                    Eigen::VectorXd jntPosePre(7);
                    jntPosePre[0] = qsPre[0];
                    jntPosePre[1] = qsPre[1];
                    jntPosePre[2] = qsPre[2];
                    jntPosePre[3] = qsPre[3];
                    jntPosePre[4] = qsPre[4];
                    jntPosePre[5] = qsPre[5];
                    jntPosePre[6] = qsPre[6];

                    Eigen::VectorXd jntPosePost(7);
                    jntPosePost[0] = qsPost[0];
                    jntPosePost[1] = qsPost[1];
                    jntPosePost[2] = qsPost[2];
                    jntPosePost[3] = qsPost[3];
                    jntPosePost[4] = qsPost[4];
                    jntPosePost[5] = qsPost[5];
                    jntPosePost[6] = qsPost[6];

                    _grasp.poseEE = P.M;
                    _grasp.posePregraspEE = Ppre.M;
                    _grasp.posePostgraspEE = Ppost.M;
                    _grasp.jntPose = jntPose;
                    _grasp.jntPregrasp = jntPosePre;
                    _grasp.jntPostgrasp = jntPosePost;
                    data.grasp = _grasp;
                    data.valid = true;
                }
            }

            return data;
        }, i, g, kin, RgripperInEE));
    }


//Waiting for the tasks to complete
    cout << "Waiting..." << endl;
    for (int i = 0; i < pSize; i++)
        futures[i].wait();

    cout << "Done!" << endl;
//Getting the Results
    int count = 0;
    for (int i = 0; i < pSize; i++)
    {
        Data data = futures[i].get();

        if (data.valid)
        {
            filteredGrasps.push_back(data.grasp);
            count++;

            if (count >= numGrasps)
                break;
        }
    }
    std::cout << "N Grasps: " << filteredGrasps.size() << endl;

    return filteredGrasps;
}

std::vector<Grasp::GraspInfo> getGrasp(dxGrasp *grasp, Kin *kin, CloudPtr& cloud, CloudNormPtr& cloud_normals, CloudPtr& cloud_features, double downSample)
{
    vision* vision = grasp->getVisionPtr();
    context c;
//CloudPtr cloud(new Cloud);
//CloudNormPtr cloud_normals(new CloudNorm);

    map<string, double> graspDurationMap = c.getGraspDurations();

//std::cout << "[Point Cloud] " << cloud->points.size() << " pts" << std::endl;
//vision->viewPointCloud(cloud);

    double radius = c.getSphereRadius();
    double radiusSearchFinger = 0.012 + 0.004;
    cloud_features.reset(new Cloud);
    std::vector<Grasp::GraspInfo> ufGrasps = grasp->getGrasps(cloud, cloud_normals, cloud_features, downSample, radius, radiusSearchFinger);

    std::cout << "[Get Grasps] " << endl;
    auto start = std::chrono::system_clock::now();
    std::vector<Grasp::GraspInfo> grasps = getFeasibleGrasps(ufGrasps, grasp, kin, true);
    auto end = std::chrono::system_clock::now();
    auto elapsed_seconds = end - start;
    std::cout << "[Duration]: " << elapsed_seconds.count() << "s\n";

    return grasps;
}

void printMsg(string message, string prep, string end)
{
    std::cout << prep << message << end;
}

void waitForUserInput(string message)
{
    std::cout << message.c_str() << endl;
    std::cin.ignore();
}

void printVec(vpColVector v, string name, string end)
{
    cout << name << " " << v.transpose() << end;
}


VecD getTargetFrame(VecD X, Eigen::Matrix4d eeRt)
{
    if (X.size() != 7)
    {
        VecD res;
        cout << "[getTargetFrame] Error" << endl;
        return res;
    }

//Convert Pose to Homogeneous matrix
    vpTranslationVector T(X[0], X[1], X[2]);
    vpQuaternionVector Q(X[3], X[4], X[5], X[6]);
    vpHomogeneousMatrix H(T, Q);

//Convert Matrix4d to Homogeneous matrix
    vpHomogeneousMatrix vpeeRt = Eigne4dTovpHomMat(eeRt);

//Forward multiplication
    vpHomogeneousMatrix Ht = H*vpeeRt;
    vpTranslationVector Tt(Ht);
    vpRotationMatrix Rt(Ht);
    vpQuaternionVector Qt(Rt);

//Get the Pose vector
    VecD Xt(7);
    Xt[0] = Tt[0];
    Xt[1] = Tt[1];
    Xt[2] = Tt[2];
    Xt[3] = Qt[0];
    Xt[4] = Qt[1];
    Xt[5] = Qt[2];
    Xt[6] = Qt[3];

    return Xt;
}


vpHomogeneousMatrix getVispHomogeneousFrmPose(VecD X)
{
    if (X.size() != 7)
    {
        cout << "[getEigenMat] Not a Pose vector" << endl;
        return vpHomogeneousMatrix();
    }

//Convert Pose to Homogeneous matrix
    vpTranslationVector T(X[0], X[1], X[2]);
    vpQuaternionVector Q(X[3], X[4], X[5], X[6]);
    vpHomogeneousMatrix H(T, Q);

    return H;
}


VecD getPoseFrmVispHomogeneous(vpHomogeneousMatrix H)
{
    vpTranslationVector T(H);
    vpRotationMatrix R(H);
    vpQuaternionVector Q(R);

//Get the Pose vector
    VecD X(7);
    X[0] = T[0];
    X[1] = T[1];
    X[2] = T[2];
    X[3] = Q[0];
    X[4] = Q[1];
    X[5] = Q[2];
    X[6] = Q[3];

    return X;
}

vpHomogeneousMatrix getCompositionTransThenRot(vpHomogeneousMatrix H1, vpHomogeneousMatrix H2)
{
    //Translate then Inplace rotation (Local axis instead of global)
    vpTranslationVector T(H2);
    vpRotationMatrix R(H2);
    vpHomogeneousMatrix Ht(T, vpRotationMatrix());
    vpHomogeneousMatrix Hr(vpTranslationVector(), R);

    //Compute the compusition between H1 and the pure translation matrix from H2
    vpHomogeneousMatrix Hc = H1*Ht;
    vpRotationMatrix Rc(Hc);
    vpTranslationVector Tc(Hc);

    //Keep the Resulting Translation matrix and Rotate inplace the axis of the found transformation
    vpRotationMatrix Rres = Rc * R;
    vpHomogeneousMatrix Hres(Tc, Rres);

    return Hres;
}

Eigen::Matrix4d getEigenMatFrmPose(VecD X)
{
    if (X.size() != 7)
    {
        cout << "[getEigenMat] Not a Pose vector" << endl;
        return Eigen::Matrix4d();
    }

//Convert Pose to Homogeneous matrix
    vpTranslationVector T(X[0], X[1], X[2]);
    vpQuaternionVector Q(X[3], X[4], X[5], X[6]);
    vpHomogeneousMatrix H(T, Q);

//Convert Matrix4d to Homogeneous matrix
    Eigen::Matrix4d He = vpHomMatToEigne4d(H);

    return He;
}


VecD getEEFrame(VecD X, Eigen::Matrix4d eeRt)
{
    if (X.size() != 7)
    {
        VecD res;
        cout << "[getEEFrame] Error" << endl;
        return res;
    }

//Convert Pose to Homogeneous matrix
    vpTranslationVector T(X[0], X[1], X[2]);
    vpQuaternionVector Q(X[3], X[4], X[5], X[6]);
    vpHomogeneousMatrix H(T, Q);

//Convert Matrix4d to Homogeneous matrix
    vpHomogeneousMatrix vptRee = Eigne4dTovpHomMat(eeRt.inverse());

//Forward multiplication
    vpHomogeneousMatrix Ht = H * vptRee;
    vpTranslationVector Tt(Ht);
    vpRotationMatrix Rt(Ht);
    vpQuaternionVector Qt(Rt);

//Get the Pose vector
    VecD Xt(7);
    Xt[0] = Tt[0];
    Xt[1] = Tt[1];
    Xt[2] = Tt[2];
    Xt[3] = Qt[0];
    Xt[4] = Qt[1];
    Xt[5] = Qt[2];
    Xt[6] = Qt[3];

    return Xt;
}

Eigen::Matrix4d vpHomMatToEigne4d(vpHomogeneousMatrix J)
{
    Eigen::Matrix4d H;
    H << J[0][0], J[0][1], J[0][2], J[0][3],
    J[1][0], J[1][1], J[1][2], J[1][3],
    J[2][0], J[2][1], J[2][2], J[2][3],
    J[3][0], J[3][1], J[3][2], J[3][3];

    return H;
}

vpHomogeneousMatrix Eigne4dTovpHomMat(Eigen::Matrix4d M)
{
    vpHomogeneousMatrix J;
    J[0][0] = M(0, 0);
    J[0][1] = M(0, 1);
    J[0][2] = M(0, 2);
    J[0][3] = M(0, 3);

    J[1][0] = M(1, 0);
    J[1][1] = M(1, 1);
    J[1][2] = M(1, 2);
    J[1][3] = M(1, 3);

    J[2][0] = M(2, 0);
    J[2][1] = M(2, 1);
    J[2][2] = M(2, 2);
    J[2][3] = M(2, 3);

    J[3][0] = M(3, 0);
    J[3][1] = M(3, 1);
    J[3][2] = M(3, 2);
    J[3][3] = M(3, 3);

    return J;
}

vpRotationMatrix getRotationMatrix(double ux, double uy, double uz, double w)
{
    vpQuaternionVector q(ux, uy, uz, w);
    vpRotationMatrix R(q);
    return R;
}

VecD getVecFrmPose(vpPoseVector p)
{
    vpTranslationVector t = p.getTranslationVector();
    vpQuaternionVector q;
    p.extract(q);

    VecD v(7);
    v[0] = t[0];
    v[1] = t[1];
    v[2] = t[2];
    v[3] = q.x();
    v[4] = q.y();
    v[5] = q.z();
    v[6] = q.w();

    return v;
}

vpPoseVector getPoseVector(std::vector<double> p)
{
    if (p.size() != 7)
        return vpPoseVector();

    vpTranslationVector t(p[0], p[1], p[2]);
    vpQuaternionVector q(p[3], p[4], p[5], p[6]);
    vpRotationMatrix r(q);

    vpPoseVector pv(t, r);
    return pv;
}

vpColVector getQuaternionError(vpQuaternionVector p1, vpQuaternionVector p2)
{
//Compute the orientation error
    double sd = p1.w();
    double sc = p2.w();

    vpColVector ud(3);
    ud[0] = p1.x();
    ud[1] = p1.y();
    ud[2] = p1.z();

    vpColVector uc(3);
    uc[0] = p2.x();
    uc[1] = p2.y();
    uc[2] = p2.z();

    double errorScalar = sc * sd + vpColVector::dotProd(ud, uc);
    vpColVector errorOrientVec(3);
    errorOrientVec = (ud*sc - uc * sd - vpColVector::cross(ud, uc))*errorScalar;
    return errorOrientVec;
}

vpColVector getPoseForceError(vpPoseVector p1, vpPoseVector p2)
{
//Get Translation
    vpTranslationVector t1 = p1.getTranslationVector();
    vpTranslationVector t2 = p2.getTranslationVector();
    vpTranslationVector te = t1 - t2;

//Get quaternion
    vpQuaternionVector q1;
    vpQuaternionVector q2;
    p1.extract(q1);
    p2.extract(q2);

    vpColVector qe = getQuaternionError(q1, q2);

    vpColVector fe(6);
    fe[0] = te[0];
    fe[1] = te[1];
    fe[2] = te[2];
    fe[3] = qe[0];
    fe[4] = qe[1];
    fe[5] = qe[2];

    return fe;
}

vpPoseVector computeSlavePose(vpPoseVector pm, vpPoseVector pmi, vpPoseVector psi)
{
    vpPoseVector pe;

//Get Translation
    vpTranslationVector tm = pm.getTranslationVector();
    vpTranslationVector tmi = pmi.getTranslationVector();
    vpTranslationVector tsi = psi.getTranslationVector();
    vpTranslationVector t = tm - tmi + tsi;

//Get the rotation matrix
    vpRotationMatrix rm;
    vpRotationMatrix rmi;
    vpRotationMatrix rsi;
    pm.extract(rm);
    pmi.extract(rmi);
    psi.extract(rsi);

//Compute the mRs (Rotation matrix of slave in master frame)
    vpRotationMatrix r = rsi * rmi.inverse()*rm;

    pe.buildFrom(t, r);

    return pe;
}

vpPoseVector computeMasterPose(vpPoseVector ps, vpPoseVector pmi, vpPoseVector psi)
{
    vpPoseVector pe;

    //Get Translation
    vpTranslationVector ts = ps.getTranslationVector();
    vpTranslationVector tmi = pmi.getTranslationVector();
    vpTranslationVector tsi = psi.getTranslationVector();
    vpTranslationVector t = ts - tsi + tmi;

    //Get the rotation matrix
    vpRotationMatrix rs;
    vpRotationMatrix rmi;
    vpRotationMatrix rsi;
    ps.extract(rs);
    pmi.extract(rmi);
    psi.extract(rsi);

    //Compute the mRs (Rotation matrix of slave in master frame)
    vpRotationMatrix r = rs.inverse()*rsi* rmi.inverse();

    pe.buildFrom(t, r);

    return pe;
}

vpColVector computeMasterForce(vpPoseVector pm, vpPoseVector ps, vpPoseVector pmi, vpPoseVector psi)
{
    vpColVector fm = getPoseForceError(ps, pm);
    vpColVector fmi = getPoseForceError(psi, pmi);
    vpColVector f = fm - fmi;

    return f;
}

vpPoseVector computePoseErrorInMaster(vpPoseVector pm, vpPoseVector ps, vpRotationMatrix RoMS)
{
    vpPoseVector pe;

//Get Translation
    vpTranslationVector tm = pm.getTranslationVector();
    vpTranslationVector ts = ps.getTranslationVector();
    vpTranslationVector te = ts - tm;

//Get the rotation matrix
    vpRotationMatrix rm;
    vpRotationMatrix rs;
    pm.extract(rm);
    ps.extract(rs);

//Compute the Rms (Rotation matrix of slave in master frame)
    vpRotationMatrix Rms = rs * RoMS*rm.inverse();

    pe.buildFrom(te, Rms);

    return pe;
}

vector<double> getPoseFromHomogeneous(Eigen::Matrix4d H)
{
    vector<double> P(7);

    vpHomogeneousMatrix J;
    J[0][0] = H(0, 0);
    J[0][1] = H(0, 1);
    J[0][2] = H(0, 2);
    J[0][3] = H(0, 3);

    J[1][0] = H(1, 0);
    J[1][1] = H(1, 1);
    J[1][2] = H(1, 2);
    J[1][3] = H(1, 3);

    J[2][0] = H(2, 0);
    J[2][1] = H(2, 1);
    J[2][2] = H(2, 2);
    J[2][3] = H(2, 3);

    J[3][0] = H(3, 0);
    J[3][1] = H(3, 1);
    J[3][2] = H(3, 2);
    J[3][3] = H(3, 3);

    vpRotationMatrix R(J);
    vpQuaternionVector Q(R);
    P[0] = J[0][3];
    P[1] = J[1][3];
    P[2] = J[2][3];
    P[3] = Q.x();
    P[4] = Q.y();
    P[5] = Q.z();
    P[6] = Q.w();

//vector<double> P;
//J.convert(P);

    return P;
}