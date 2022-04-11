1.publish data in ros topic 
  https://github.com/irapkaist/file_player_mulran
  cd backup/software/mulran_player  
  source devel/setup.bash
  roslaunch file_player file_player.launch
  load backup/dataset/lt_mapper/parkinglot/01
2.generate .pcd file using sc-lio-sam
  https://github.com/gisbi-kim/SC-LIO-SAM
  cd /scliosam_ws/src/SC-LIO-SAM/SC-LIO-SAM/config  edit savepcdDictionary for each session
  source devel/setup.bash
  roslaunch lio_sam run_mulran.launch
  for the dataset 01, skip the first few seconds for the data is not good
  generate the session data for the sequence 02 too.
  01 and 02 started at the different poses, diffenrent trajectories
  use cloudcompare.CloudComapare to compare .pcd files
3. use LT-SLAM to align 01 and 02
   https://github.com/gisbi-kim/lt-mapper#release-schedule
   specify the sequences database paths in mapper_ws/src/lt-mapper/ltslam/config/params.yaml by editing sessions_dir and central_sess_name and query_sess_name
   and specify save_dictionary
   roslaunch ltslam run.launch
   LT-SLAM first detect and stitch scan-context-based loops 
   and get rough initial transformation
   then finely stitch the two trajectories using radius-search(i.e., proximity)
   the results are stored in _central_aft_intersession_loops.txt
   central means they are in the shared coodinate called central
   compare them in cloudcompare.CloudComapare the 4 8 12 columns are X Y Z  just like the matrix [R|t]
4. generate merged map
   use /home/ding/scliosam_ws/src/SC-LIO-SAM/SC-LIO-SAM/tools/python/makeMergedMap.py
   edit the data_dir and scan_idx_range_to_stack 
   note: optimized_poses.txt is saved in mapoptimization so it's not the lt-slam pose but the original pose
   to generate a map using saved keyframes and optimized poses for 01 and 02(seperated)
   finally we have two point cloud in the same frame

fdp PoseGraphTest.dot -Tpdf > graph.pdf  //recommand
dot test.dot -Tpng -o test.png //may be fuzzy

//todo 
  the small drift will be handled in LT-removert to detect changes
  the noisy dynamic points (as ghosts) will be removed within LT-removert before detecting changes   

  
