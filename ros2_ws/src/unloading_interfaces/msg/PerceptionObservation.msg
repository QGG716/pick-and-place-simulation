string schema_version
string observation_id
string source_epoch
uint64 source_sequence
builtin_interfaces/Time capture_time
builtin_interfaces/Time processed_time
string clock_domain
string provider
string upstream_commit
string model_identity
string config_identity
string status
string failure_code
string failure_message
bool synthetic
unloading_interfaces/CargoObservation[] cargo
unloading_interfaces/UnknownRegion[] unknown_regions
string coverage_json
# Preserve source double timestamps through nanosecond ROS transport without
# rewriting capture history. Exact values must quantize to the Time fields.
bool has_exact_source_times
float64 exact_capture_time_s
float64 exact_processed_time_s
