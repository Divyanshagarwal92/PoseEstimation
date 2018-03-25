/*
  - We consider a given set of 3D -> 2D model-to-image correspondences, i.e. a
  set of pairs [X_i,Y_i,Z_i] -> [u_i,v_i]

  - We assume a pin-hole camera model, with focal length in pixels [fx, fy],
  principal point [cx, cy] and no distortions.

  - Our goal is to estimate the optimal 3D pose (in the least squares sense)
  of the model in the image.

  --------------

  Intrinsics: 
  fx = 510 ; fy = 500 ; cx = 320 ; cy = 240

  --------------

  Correspondence Data: 

  X_i       Y_i        Z_i        u_i         v_i
   0.401191  -0.533441  3.653708  345.623230  129.274246 
   1.707242  -0.330801  3.120764  557.206604  163.980057 
   0.495830   1.350876  3.327281  347.657654  396.280151 
  -0.480684  -0.240581  1.522665  126.022064  116.588417 
  -0.901928   0.722427  2.821982  117.406204  317.955231 
   0.660500   0.167810  1.233900  532.955627  284.710297 
  -0.279028   0.177209  2.684985  231.168579  230.508911 
   0.497133   0.471177  2.243698  387.624725  308.307556 
  -0.383256  -0.657891  2.383663  209.122742  55.681664  
   0.149257  -0.735220  1.856616  336.905609  5.932389 

   --------------

*/

Objective: 

Write a program in C or C++ that estimates the optimal 3D pose (in the least squares sense) of the model in the image, assuming the model 3D coordinates were given at identity. The program should be run on the sample data  provided in the file, and return the 3D rotation and translation of the model in the image. You can use basic algebra (matrix operations) from existing libraries, but the optimization itself has to be written from scratch. You can assume that the initial pose is not very far from identity.



End time: 5:30PM, 03/24
g++ -I eigen-eigen-5a0156e40feb/ simple_eigen.cpp -o simple_eigen
