# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.mpc.Debug:
/Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/Debug/mpc:
	/bin/rm -f /Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/Debug/mpc


PostBuild.mpc.Release:
/Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/Release/mpc:
	/bin/rm -f /Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/Release/mpc


PostBuild.mpc.MinSizeRel:
/Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/MinSizeRel/mpc:
	/bin/rm -f /Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/MinSizeRel/mpc


PostBuild.mpc.RelWithDebInfo:
/Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/RelWithDebInfo/mpc:
	/bin/rm -f /Users/huapinggu/Sources/NDCar/term2/CarND-MPC-Project/RelWithDebInfo/mpc




# For each target create a dummy ruleso the target does not have to exist
