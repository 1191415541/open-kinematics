defaults command_file echo_commands=off
variable set variable=.ACAR.variables.errorFlag integer=0
acar files assembly open &
 assembly_name="<acar_concept>/assemblies.tbl/Demo_Vehicle_Variants.asy" &
 variant=suspfront &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension parallel_travel submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_probe" &
 output_suffix="parallel" &
 nsteps=4 &
 bump_disp=10 &
 rebound_disp=-10 &
 stat_steer_pos=0 &
 load_results=yes &
 vertical_setup=wheel_center_height &
 vertical_input=wheel_center_height &
 vertical_type=relative &
 steering_input=length &
 log_file=yes &
 analysis_mode=interactive &
 create_report=yes &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension compliance submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_probe" &
 output_suffix="com" &
 nsteps=34 &
 load_results=yes &
 vertical_input=wheel_center_height &
 wheel_fixed_height=0 &
 fore_force_wc=500 &
 aft_force_wc=500 &
 fore_force_cp=500 &
 aft_force_cp=500 &
 lat_force_cp=500 &
 align_torq_wc=1000 &
 lat_force_offset=0 &
 steering_input=length &
 log_file=yes &
 analysis_mode=interactive &
 error_variable=.ACAR.variables.errorFlag
acar analysis report &
 analysis="suspension_mbd_probe_com" &
 report_template="comptest.rtp" &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension static_load submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_probe" &
 output_suffix="static_load" &
 nsteps=4 &
 steer_upper=0 &
 steer_lower=0 &
 load_results=yes &
 steering_input=length &
 vertical_setup=wheel_center_height &
 vertical_input=wheel_center_height &
 vertical_type=relative &
 later_for_upr_l=500 &
 later_for_upr_r=500 &
 later_for_lwr_l=-500 &
 later_for_lwr_r=-500 &
 log_file=yes &
 coordinate_system=vehicle &
 analysis_mode=interactive &
 error_variable=.ACAR.variables.errorFlag
acar analysis report &
 analysis="suspension_mbd_probe_static_load" &
 report_template="knc_front_static.rtp" &
 error_variable=.ACAR.variables.errorFlag
file text open file="probe_status.txt" open=overwrite
file text write format="error=%d" value=(eval(.ACAR.variables.errorFlag))
file text close
exit confirm=yes
