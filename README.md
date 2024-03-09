# ForceBalance in MATLAB (to be migrated)


![alt text](https://github.com/teampolar/ForceBalance/blob/production/BeddingRender_Framnes.png?raw=true)

Hello there

- First of all, stop with what you are doing and download the bedmachineantarctica data (it's on the Force balance folder in the energy management sharepoint https://teampolar.sharepoint.com/:f:/s/energymanagement/Eh3ete1hfTdLisalpbV3F24BPKMdoDVae7QeEpzpvdbjKg?e=nfVV2E). It's about 900 mb so should take a while. Put it in one folder together with all the other force balance files so matlab can access it.  

About the bedmachine dataset : "This data set, part of the NASA Making Earth System Data Records for Use in Research Environments (MEaSUREs) program, contains a bed topography/bathymetry map of Antarctica based on mass conservation, streamline diffusion, and other methods. The data set also includes ice thickness, surface elevation, an ice/ocean/land mask, ice thickness estimation errors, and a map showing where each method was utilized."

This script is set up to look at the Framnes mountains only (one of Saef's locations). However, it can easily be changed to whatever location by calling the SCARLOC function and changing the lat + lon accordingly!

- To utilize these datasets, the following matlab add-on's need to be installed, all made by Chad Greene.
1. Antarctic boundaries, grounding line, and masks from InSAR
2. Antarctic mapping tools
3. Bedmachine
4. Cmocean perceptually-uniform colormaps (nice mappings)
        
- Clone this repo to the folder all the to-be accessed datasets are in.
- mass_est_rollfricimproved.slx is the latest version of the simulink model (it inputs parameters into the force calculations and outputs a state of charge and/or power usage along the route).
- elevation_script.mlx contains the script for selecting routes, velocity profiles and parameter inputs that will be fed into the simulink model.
- The pictures are to be consulted during route selection (**THIS WILL BE IMPROVED ON**)
- 
Now everything is up-to-date. Check out the script and simulink model. Comments are made so going over it gives a good idea of what is done.

 **The outputs of this script is the final State of Charge (SoC) after having driven the selected route, given a certain battery mass and total vehicle mass.**

If you have any further questions about this model you can ask jord and cas. Cheers, xx
