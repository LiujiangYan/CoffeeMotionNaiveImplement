import string,time
from pylab import *
from numpy import *
from openravepy import *
from TOPP import TOPPbindings
from TOPP import TOPPpy
from TOPP import TOPPopenravepy
from TOPP import Trajectory
from TOPP import Utilities

dof = 3
inputMat = numpy.loadtxt(open("input.csv","rb"), delimiter=",",skiprows=0)

#trajectory
trajectorystring = ""
ds = inputMat[0,12]
for i in range(20):
	trajectorystring += "%f\n%d"%(ds,dof)
	for j in range(dof):
		q0 = inputMat[i,j]
		q1 = inputMat[i,j+3]
		qd0 = inputMat[i,j+6]
		qd1 = inputMat[i,j+9]
 		a,b,c,d = Utilities.Interpolate3rdDegree(q0,q1,qd0,qd1,ds)
		trajectorystring += "\n%f %f %f %f"%(d,c,b,a)
	trajectorystring += "\n"
print "interpolate"
traj = Trajectory.PiecewisePolynomialTrajectory.FromString(trajectorystring)

print "trajecotry"

#constraints
vmax = 1*ones(dof)
amax = 5*ones(dof)

# Set up the TOPP instance
discrtimestep = 0.005
uselegacy = True
t0 = time.time()
if uselegacy: #Using the legacy KinematicLimits (a bit faster but not fully supported)
    constraintstring = str(discrtimestep)
    constraintstring += "\n" + string.join([str(v) for v in vmax])
    constraintstring += "\n" + string.join([str(a) for a in amax])
    x = TOPPbindings.TOPPInstance(None,"KinematicLimits",constraintstring,trajectorystring);
else: #Using the general QuadraticConstraints (fully supported)
    constraintstring = str(discrtimestep)
    constraintstring += "\n" + string.join([str(v) for v in vmax])
    constraintstring += TOPPpy.ComputeKinematicConstraints(traj, amax, discrtimestep) 
    x = TOPPbindings.TOPPInstance(None,"QuadraticConstraints",constraintstring,trajectorystring);
print "Topp set up"

# Run TOPP
t1 = time.time()
ret = x.RunComputeProfiles(0,0)
x.ReparameterizeTrajectory()
t2 = time.time()

print "Using legacy:", uselegacy
print "Discretization step:", discrtimestep
print "Setup TOPP:", t1-t0
print "Run TOPP:", t2-t1
print "Total:", t2-t0

# Display results
ion()
x.WriteProfilesList()
x.WriteSwitchPointsList()
profileslist = TOPPpy.ProfilesFromString(x.resprofilesliststring)
switchpointslist = TOPPpy.SwitchPointsFromString(x.switchpointsliststring)
TOPPpy.PlotProfiles(profileslist,switchpointslist,4)
x.WriteResultTrajectory()
traj1 = Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
dtplot = 0.01
#TOPPpy.PlotKinematics(traj,traj1,dtplot,vmax,amax)

#the raw data of parameterized joint variable
tvect = arange(0, traj1.duration + dtplot, dtplot)
qvect = array([traj1.Eval(t) for t in tvect])
numpy.savetxt('data/time.csv', tvect, delimiter=',')
numpy.savetxt('data/jointVariable.csv', qvect, delimiter=',')

#velocity
qdvect = array([traj1.Evald(t) for t in tvect])
numpy.savetxt('data/jointVel.csv', qdvect, delimiter=',')

#acceleration
qddvect = array([traj1.Evaldd(t) for t in tvect])
numpy.savetxt('data/jointAccl.csv', qddvect, delimiter=',')

raw_input()