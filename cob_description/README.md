## Urdf versions

This package contains the description of the components of Care-O-bot
robots, inside the folder urdf you can find different models and
versions, the following table explain the distribution:

### Torso

  -- -------------------- -- ------------------------------------------------ -- --------------------------- --
     **Version number**      **description**                                     **used in robot**           
     torso_v0                Torso with 2 PW (2 pan links and 2 tilt links)      cob3-1 and cob3-2           
     torso_v1                Torso with 3PRL (1 pan link and 2 tilt links)       cob3-3, cob3-4 and cob3-6   
  -- -------------------- -- ------------------------------------------------ -- --------------------------- --

Each joint has predefined limits:

  -- -------------------- -- ----------------- -- ------------------- -- ----------------- -- ------------------- -- ----------------- -- ------------------- --
                                                  **torso_1_joint**                           **torso_2_joint**                           **torso_3_joint**   
                             **lower_limit**      **upper_limit**        **lower_limit**      **upper_limit**        **lower_limit**      **upper_limit**     
     **urdf limit**          -0,22                0,22                   -0,22                0,22                   -0,37                0,37                
     **hardware limit**      -0,25                0,25                   -0,25                0,25                   -0,40                0,40                
  -- -------------------- -- ----------------- -- ------------------- -- ----------------- -- ------------------- -- ----------------- -- ------------------- --

### Tray

  -- ---------------- -- --------------------------- -- ------------------------------------------- --
     **Version**         **description**                **used in robot**                           
     tray                Tray with 1 PRL                cob3-1, cob3-2, cob3-3, cob3-4 and cob3-7   
     tray_powerball      Tray with 1 PRL and 1 ERB      cob3-5 and cob3-6                           
  -- ---------------- -- --------------------------- -- ------------------------------------------- --

Each joint has predefined limits:

#### tray (1 DOF)

  -- -------------------- -- ----------------- -- ------------------ --
                                                  **tray_1_joint**   
                             **lower_limit**      **upper_limit**    
     **urdf limit**          -3,3                 0,2                
     **hardware limit**      -3,33                0,23               
  -- -------------------- -- ----------------- -- ------------------ --

#### tray_powerball (3 DOF)

  -- -------------------- -- ----------------- -- ------------------ -- ----------------- -- ------------------ -- ----------------- -- ------------------ --
                                                  **tray_1_joint**                           **tray_2_joint**                           **tray_3_joint**   
                             **lower_limit**      **upper_limit**       **lower_limit**      **upper_limit**       **lower_limit**      **upper_limit**    
     **urdf limit**          -1,37                0,02                  -3,16                3,16                  -3,16                3,16               
     **hardware limit**      -1,4                 0,05                  -3,19                3,19                  -3,19                3,19               
  -- -------------------- -- ----------------- -- ------------------ -- ----------------- -- ------------------ -- ----------------- -- ------------------ --

### Head

  -- -------------------- -- --------------------------------------------------- -- --------------------------- --
     **Version number**      **description**                                        **used in robot**           
     head_v1                 Head with 2 AVT Pike C145 and Kinect                   cob3-1                      
     head_v2                 Head with AVT Prosilica GigE and Mesa Swissranger      cob3-2                      
     head_v3                 Head with 2 AVT Prosilica GigE and Kinect              cob3-3, cob3-4 and cob3-6   
  -- -------------------- -- --------------------------------------------------- -- --------------------------- --

In the package
[cob_hardware_config](https://index.ros.org/p/cob_hardware_config) you
can find for each robot the urdf file that load the component
descriptions together.

## Generate stl file

We used Solidworks to generate stl files. Please pay attention to the
following issues while generating new stl file:

- define a new reference coordinate system in SolidWorks and export the
  stl file using this reference coordinate system.
- the mesh size (amount of triangles) is limited to ?32000?
- the stl file should not begin with "solid", therefore substitute the
  string "solid" e.g. with "robot" using 

```
sed -i '0,/RE/s/solid/robot/' */*.stl
```
