# Policy
$V_{sp} = clamp(K_p(p_{wp} - p_{current}), v_{max})$

Keeping:
    1. original wapoints
    2. use std::clamp to clamp the veloctiy commands, be aware of NED frame so negative z vel is up, also clamp only takes single var no STL container
    3. 