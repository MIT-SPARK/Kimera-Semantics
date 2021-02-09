DATASET_PATH="$HOME/datasets/uHumans2/"
ROSBAGS=(
  #"subway_scene/uHumans2_subway_s1_00h"
  #"subway_scene/uHumans2_subway_s1_24h"
  #"subway_scene/uHumans2_subway_s1_36h"
  #"apartment_scene/uHumans2_apartment_s1_00h"
  "apartment_scene/uHumans2_apartment_s1_01h"
  "apartment_scene/uHumans2_apartment_s1_02h"
  #"office_scene/uHumans2_office_s1_00h"
  #"office_scene/uHumans2_office_s1_06h"
  #"office_scene/uHumans2_office_s1_12h"
  #"neighborhood_scene/uHumans2_neighborhood_s1_00h"
  #"neighborhood_scene/uHumans2_neighborhood_s1_24h"
  #"neighborhood_scene/uHumans2_neighborhood_s1_36h"
)
CSVS=(
  #"tesse_multiscene_underground1_segmentation_mapping.csv"
  #"tesse_multiscene_underground1_segmentation_mapping.csv"
  #"tesse_multiscene_underground1_segmentation_mapping.csv"
  #"tesse_multiscene_archviz1_segmentation_mapping.csv"
  "tesse_multiscene_archviz1_segmentation_mapping.csv"
  "tesse_multiscene_archviz1_segmentation_mapping.csv"
  #"tesse_multiscene_office2_segmentation_mapping.csv"
  #"tesse_multiscene_office2_segmentation_mapping.csv"
  #"tesse_multiscene_office2_segmentation_mapping.csv"
  #"tesse_multiscene_neighborhood1_segmentation_mapping.csv"
  #"tesse_multiscene_neighborhood1_segmentation_mapping.csv"
  #"tesse_multiscene_neighborhood1_segmentation_mapping.csv"
)
LOGS_PATH="$HOME/Code/ROS/kimera_ws/src/Kimera-Semantics/kimera_semantics_ros/mesh_results"
OUTPUT_PATH="$HOME/Documents/uHumans2_VIO_vxblx"

DYNAMIC_MASKING=0

i=0
for ROSBAG in ${ROSBAGS[@]}
do
  ROSBAG_PATH="${DATASET_PATH}${ROSBAG}.bag"

  if $DYNAMIC_MASKING
  then
    echo "Running Dynamic Masking!"

    roslaunch kimera_semantics_ros kimera_semantics_rosbag.launch \
      rosbag_path:=$ROSBAG_PATH \
      use_dynamic_masking:="true" \
      base_link_frame:="base_link_DVIO" \
      semantic_label_2_color_csv_filename:="${CSVS[i]}"
    mv $LOGS_PATH/mesh.ply "$OUTPUT_PATH/$ROSBAG/mesh_DVIO.ply"
    mv $LOGS_PATH/tsdf_esdf_layers.vxblx "$OUTPUT_PATH/$ROSBAG/tsdf_esdf_DVIO.vxblx"

    roslaunch kimera_semantics_ros kimera_semantics_rosbag.launch \
      rosbag_path:=$ROSBAG_PATH \
      use_dynamic_masking:="true" \
      base_link_frame:="base_link_gt" \
      semantic_label_2_color_csv_filename:="${CSVS[i]}"
    mv $LOGS_PATH/mesh.ply "$OUTPUT_PATH/$ROSBAG/mesh_gt.ply"
    mv $LOGS_PATH/tsdf_esdf_layers.vxblx "$OUTPUT_PATH/$ROSBAG/tsdf_esdf_gt.vxblx"
  else
    echo "Running WITHOUT Dynamic Masking!"

    roslaunch kimera_semantics_ros kimera_semantics_rosbag.launch \
      rosbag_path:=$ROSBAG_PATH \
      use_dynamic_masking:="false" \
      base_link_frame:="base_link_DVIO" \
      semantic_label_2_color_csv_filename:="${CSVS[i]}"
    mv $LOGS_PATH/mesh.ply "$OUTPUT_PATH/$ROSBAG/mesh_DVIO_wo_DM.ply"
    mv $LOGS_PATH/tsdf_esdf_layers.vxblx "$OUTPUT_PATH/$ROSBAG/tsdf_esdf_DVIO_wo_DM.vxblx"

    roslaunch kimera_semantics_ros kimera_semantics_rosbag.launch \
      rosbag_path:=$ROSBAG_PATH \
      use_dynamic_masking:="false" \
      base_link_frame:="base_link_gt" \
      semantic_label_2_color_csv_filename:="${CSVS[i]}"
    mv $LOGS_PATH/mesh.ply "$OUTPUT_PATH/$ROSBAG/mesh_gt_wo_DM.ply"
    mv $LOGS_PATH/tsdf_esdf_layers.vxblx "$OUTPUT_PATH/$ROSBAG/tsdf_esdf_gt_wo_DM.vxblx"
  fi

  ((i++))

done
