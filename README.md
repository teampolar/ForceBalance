# ForceBalance

Hello there

- First of all, stop with what you are doing and download the bedmachineantarctica data (it's on the Force balance folder in the energy management sharepoint https://teampolar.sharepoint.com/:f:/s/energymanagement/Eh3ete1hfTdLisalpbV3F24BPKMdoDVae7QeEpzpvdbjKg?e=nfVV2E). It's about 900 mb so should take a while. Put it in one folder together with all the other force balance files so matlab can access it.  

About the bedmachine dataset : "This data set, part of the NASA Making Earth System Data Records for Use in Research Environments (MEaSUREs) program, contains a bed topography/bathymetry map of Antarctica based on mass conservation, streamline diffusion, and other methods. The data set also includes ice thickness, surface elevation, an ice/ocean/land mask, ice thickness estimation errors, and a map showing where each method was utilized."

- To utilize these datasets, the following matlab add-on's need to be installed, all made by Chad Greene.
1. Antarctic boundaries, grounding line, and masks from InSAR
2. Antarctic mapping tools
3. Bedmachine
        
- Download energymodeling-production.zip and extract it to that folder

Now everything is up-to-date. Check out the various scripts and simulink models. Comments are made so going over it gives a good idea of what is done. NOTE THAT THIS IS NOT DONE YET AND EVERYTHING NEEDS TO BE INTEGRATED TOGETHER.

Elevationscript is the building block from which everything afterwards is determined. For now, it includes the driving route selection and velocity profiler for the Framnes mountains (one of Saef's locations). The outputs of this script are the velocity profile for the distance intervals and the slopes. These are (for now) fed into the massestimate simulink model. 

If you have any further questions about life or this model you can ask jord and cas. xx
