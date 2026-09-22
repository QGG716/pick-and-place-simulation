# ExecutionContext contract 1.2.0; older publishers must be rebuilt and upgraded.
string schema_version
# Transport identity, independent of execution epoch and planning_generation.
string publisher_epoch
uint64 publisher_sequence
# A takeover requires a new, non-retired publisher_epoch and sequence zero.
bool publisher_restart
string session_id
string epoch
uint64 planning_generation
string allowed_plan_id
string predecessor_plan_id
builtin_interfaces/Time observed_time
string clock_domain
