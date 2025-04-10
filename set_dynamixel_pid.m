% This function is used to change the PID gains of the dynamixel servos
function set_dynamixel_pid(id, Kp, Ki, Kd)
    global Px100;


    if ~ismember(id, Px100.DXL_ID)
        error("Invalid motor ID: %d. It must be one of: %s", id, mat2str(Px100.DXL_ID));
    end

    fprintf("Setting PID for motor %d -> D: %d, I: %d, P: %d\n", id, Kd, Ki, Kp);

    write2ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, id, 80, uint16(Kd));
    write2ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, id, 82, uint16(Ki));
    write2ByteTxRx(Px100.port_num, Px100.PROTOCOL_VERSION, id, 84, uint16(Kp));

    disp("PID gains sent to motor.");
end


