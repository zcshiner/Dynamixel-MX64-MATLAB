%MX64 A wrapper class to interface with Robotis's USB2Dynamixel
%     and control MX64 series servos using communication version 1.0.
%   
%     API Info     http://support.robotis.com/en/software/dynamixel_sdk/usb2dynamixel.htm
%     Servo Manual http://support.robotis.com/en/product/dynamixel/mx_series/mx-64.htm

%     Created: April 2015
%     Author: Zachary Shiner
%
%     Licensed under the MIT license.

classdef MX64

    properties (SetAccess = immutable, GetAccess = public)
        port;
        baudCode;
    end

    properties (SetAccess = immutable, GetAccess = private)
		cleanup;
    end

	properties (Constant, Access = private)
	% Dynamixel MX64 Control Table
		% EEPROM AREA
		EEPROM_MODEL_NUMBER_L         = 0;
		EEPROM_MODEL_NUMBER_H         = 1;
		EEPROM_VERSION                = 2;
		EEPROM_ID                     = 3;
		EEPROM_BAUD_RATE              = 4;
		EEPROM_RETURN_DELAY_TIME      = 5;
		EEPROM_CW_ANGLE_LIMIT_L       = 6;
		EEPROM_CW_ANGLE_LIMIT_H       = 7;
		EEPROM_CCW_ANGLE_LIMIT_L      = 8;
		EEPROM_CCW_ANGLE_LIMIT_H      = 9;
		EEPROM_LIMIT_TEMPERATURE      = 11;
		EEPROM_LOW_LIMIT_VOLTAGE      = 12;
		EEPROM_HIGN_LIMIT_VOLTAGE     = 13;
		EEPROM_MAX_TORQUE_L           = 14;
		EEPROM_MAX_TORQUE_H           = 15;
		EEPROM_RETURN_LEVEL           = 16;
		EEPROM_ALARM_LED              = 17;
		EEPROM_ALARM_SHUTDOWN         = 18;
		EEPROM_TURN_OFFSET_L		  = 20;
		EEPROM_TURN_OFFSET_H		  = 21;
		EEPROM_RESOLUTION_DIVIDER	  = 22;
		% RAM AREA
		RAM_TORQUE_ENABLE             = 24;
		RAM_LED                       = 25;
		RAM_DERIVATIVE_GAIN           = 26;
		RAM_INTERGRAL_GAIN            = 27;
		RAM_PROPORTIONAL_GAIN         = 28;
		RAM_GOAL_POSITION_L           = 30;
		RAM_GOAL_POSITION_H           = 31;
		RAM_GOAL_SPEED_L              = 32;
		RAM_GOAL_SPEED_H              = 33;
		RAM_TORQUE_LIMIT_L            = 34;
		RAM_TORQUE_LIMIT_H            = 35;
		RAM_PRESENT_POSITION_L        = 36;
		RAM_PRESENT_POSITION_H        = 37;
		RAM_PRESENT_SPEED_L           = 38;
		RAM_PRESENT_SPEED_H           = 39;
		RAM_PRESENT_LOAD_L            = 40;
		RAM_PRESENT_LOAD_H            = 41;
		RAM_PRESENT_VOLTAGE           = 42;
		RAM_PRESENT_TEMPERATURE       = 43;
		RAM_REGISTER                  = 44;
		RAM_MOVING                    = 46;
		RAM_LOCK                      = 47;
		RAM_PUNCH_L                   = 48;
		RAM_PUNCH_H                   = 49;
		RAM_CURRENT_L				  = 68;
		RAM_CURRENT_H			      = 69;
		RAM_TORQUE_CONTROL_MODE		  = 70;
		RAM_GOAL_TORQUE_L			  = 71;
		RAM_GOAL_TORQUE_H			  = 72;
		RAM_GOAL_ACCELERATION		  = 73;


	% Communication v1.0 Command Table
		COMMAND_PING                  = 1;
		COMMAND_READ_DATA             = 2;
		COMMAND_WRITE_DATA            = 3;
		COMMAND_REG_WRITE_DATA        = 4;
		COMMAND_ACTION                = 5;
		COMMAND_RESET                 = 6;
		COMMAND_SYNC_WRITE            = 131;

	% Specials
		BROADCAST_ID        = 254;
        
    % Error Bits
        ERRBIT_VOLTAGE      = 1;
        ERRBIT_ANGLE        = 2;
        ERRBIT_OVERHEAT     = 4;
        ERRBIT_RANGE        = 8;
        ERRBIT_CHECKSUM     = 16;
        ERRBIT_OVERLOAD     = 32;
        ERRBIT_INSTRUCTION  = 64;
        
    % Comm Status Values
        COMM_TXSUCCESS      = 0;
        COMM_RXSUCCESS      = 1;
        COMM_TXFAIL         = 2;
        COMM_RXFAIL         = 3;
        COMM_TXERROR        = 4;
        COMM_RXWAITING      = 5;
        COMM_RXTIMEOUT      = 6;
        COMM_RXCORRUPT      = 7;
        
	end


    methods (Access = public)
        function obj = MX64(port, baud)
        %MX64.MX64 Constructor.  Initialize with port number and baudrate.
        %
        %   Dynamixel = MX64(port,baud) returns an object to control Dynamixel
        %   MX64 servos.

            % record port for later
            obj.port = port;

            % decypher baud rate into the appropriate baud code to send
            switch (baud)
                case 1000000 % 1Mb
                    obj.baudCode = 1;
                case 2250000
                    obj.baudCode = 250;
                case 2500000
                    obj.baudCode = 251;
                case 3000000
                    obj.baudCode = 252;
                otherwise
                    obj.baudCode = (2000000/baud) - 1;
            end

            % load the SDK library.  MUST BE ADDED TO CURRENT PATH PRIOR TO INITIALIZING.
			% example: addpath('.\dxl_sdk_win32_v1_02\bin', '.\dxl_sdk_win32_v1_02\import')
            loadlibrary('dynamixel','dynamixel.h')

            % Check to see if the initialization was done correctly (1=PASS 0=FAIL)
            if calllib('dynamixel','dxl_initialize',obj.port,obj.baudCode) == 0
                error('Failed to open USB2dynamixel!');
            end
            disp('Success connecting to USB2dynamixel!');

            % call the destructor when this object is deleted
            obj.cleanup = onCleanup(@()delete(obj));
        end


        function delete(obj)
        % Close and unload the dynamixel library

            disp('Closing connection...');
            calllib('dynamixel','dxl_terminate');
            unloadlibrary('dynamixel');
        end


        function led(obj,ID,state)
        %MX64.led Set the state of the led on a dynamixel actuator
        %
        %   Dynamixel.led(ID, state)

			if state == 0 || state == 1
				calllib('dynamixel','dxl_write_byte',ID,obj.RAM_LED,state);
			else
				warning('Parameter for led out of bounds.  Must be 0 or 1.');
			end
        end


        function position(obj,ID,position)
        %MX64.position Set the position of the servo in joint mode.  Both CW/CCW limits must != 0
        %
        %   Dynamixel.position(id, position)

            calllib('dynamixel','dxl_write_word',ID,obj.RAM_GOAL_POSITION_L,position);
        end

        function syncWritePosition(obj,varargin)
		%MX64.syncWritePosition Set the position of multiple consecutive actuators simultaneously.
        %
        %   Dynamixel.syncWritePosition([pos1, pos2, ...])

			% first argument given should be a vector of motor positions
			motorPositions = varargin{1};

            % how many motors do we have?
			motorCount = length(motorPositions);

            calllib('dynamixel','dxl_set_txpacket_id',obj.BROADCAST_ID);					% Broadcast ID
            calllib('dynamixel','dxl_set_txpacket_instruction',obj.COMMAND_SYNC_WRITE);		% Sync Command
            calllib('dynamixel','dxl_set_txpacket_parameter',0,obj.RAM_GOAL_POSITION_L);	% Start address to write to

            paramNumber = 3; % three write instructions in the following loop
            calllib('dynamixel','dxl_set_txpacket_parameter',1,paramNumber - 1);    	% Length of sub-packet

			% loop over each motor position given
			for id = 1:motorCount

				% extract an individual motor angle and make sure its an integer
                angle = motorPositions(id);

                calllib('dynamixel','dxl_set_txpacket_parameter',(3*id)-1,id);          % ID of motor we want to address
                lowByte = calllib('dynamixel','dxl_get_lowbyte',angle);                 % calculate low byte (LSB) of 2 byte word
                highByte = calllib('dynamixel','dxl_get_highbyte',angle);               % calculate high byte (MSB) of 2 byte word
                calllib('dynamixel','dxl_set_txpacket_parameter',3*id,lowByte);         % write low byte
                calllib('dynamixel','dxl_set_txpacket_parameter',(3*id)+1,highByte);    % write high byte
            end

            % length = (L+1) X N + 4 | L = length of subPacket, N = number of servos
            calllib('dynamixel','dxl_set_txpacket_length',(paramNumber*motorCount)+4);  % Length of entire packet
            calllib('dynamixel','dxl_txrx_packet');                                     % Transmit packet
        end


        function setSpeed(obj,ID,speed)
        %MX64.setSpeed Set the speed of a servo by ID
        %
        %   Dynamixel.setSpeed(ID, speed)

            calllib('dynamixel','dxl_write_word',ID,obj.RAM_GOAL_SPEED_L,speed);
        end

        function setAngleLimit(obj,ID,CW,CCW)
        %MX64.setAngleLimit Set angle limits in servo EEPROM.
        %
        %   Dynamixel.setAngleLimit(ID, CC Limit, CCW Limit)
        %     CW = minimum, CCW = maximum.
        %     Wheel Mode: both at 0
        %     Joint Mode: neither at 0
        %     Multi-turn Mode: both at 4095

            if CCW >= CW
                calllib('dynamixel','dxl_write_word',ID,obj.EEPROM_CW_ANGLE_LIMIT_L,CW);
                calllib('dynamixel','dxl_write_word',ID,obj.EEPROM_CCW_ANGLE_LIMIT_L,CCW);
            else
                error('CCW value must be greater than or equal to CW value.');
            end
        end
        
        function torqueEnable(obj,ID,state)
        %MX64.torqueEnable Enable/disable torque generation. Boolean.
        %
        %   Dynamixel.torqueEnable(ID, enable)
        
            if state == 0 || state == 1
                calllib('dynamixel','dxl_write_byte',ID,obj.RAM_TORQUE_ENABLE,state);
            else
                error('Parameter for torqueEnable out of bounds.  Must be 0 or 1.');
            end
        end

        function setPID(obj,ID,P,I,D)
        %MX64.setPID Sets PID values in RAM for specified servo
        %
        %   Dynamixel.setPID(ID, P, I, D)

            calllib('dynamixel','dxl_write_byte',ID,obj.RAM_DERIVATIVE_GAIN,D);
            calllib('dynamixel','dxl_write_byte',ID,obj.RAM_INTERGRAL_GAIN,I);
            calllib('dynamixel','dxl_write_byte',ID,obj.RAM_PROPORTIONAL_GAIN,P);
        end
        
        
        function state = isMoving(obj,ID)
        %MX64.isMoving See if Goal Position command execution is completed or in progress.
        %
        %   boolean = Dynamixel.isMoving(ID)
            
            packetReturn = calllib('dynamixel','dxl_read_byte',ID,obj.RAM_MOVING);
            obj.checkRXerror;
            state = packetReturn;
        end
        
        
        function position = getPresentPosition(obj,ID)
        %MX64.getPresentPosition Retrives the present position of a specified servo.
        %
        %   position = Dynamixel.getPresentPosition(ID)
        
            packetReturn = calllib('dynamixel','dxl_read_word',ID,obj.RAM_PRESENT_POSITION_L);
            obj.checkRXerror
            position = packetReturn;
        end

        function waitMoveComplete(obj,numMotors)
        %MX64.waitMoveComplete Waits for n mumber of motors to stop moving
            
            for n = 1:numMotors
                while obj.isMoving(n) == 1;
                    % just wait.
                end
            end
        end
        
        function errorCodes = printErrorCode(obj)
        %MX64.errorCodes() Prints out one or many error codes given by the
        %  return packet.  Also returns the sum of return error bits.
            errorCodes = 0;
            if calllib('dynamixel','dxl_get_rxpacket_error', obj.ERRBIT_VOLTAGE) == 1
                warning('Input Voltage Error!');
                errorCodes = errorCodes + obj.ERRBIT_VOLTAGE;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_ANGLE) == 1
                warning('Angle limit error!');
                errorCodes = errorCodes + obj.ERRBIT_ANGLE;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_OVERHEAT) == 1
                warning('Overheat error!');
                errorCodes = errorCodes + obj.ERRBIT_OVERHEAT;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_RANGE) == 1
                warning('Out of range error!');
                errorCodes = errorCodes + obj.ERRBIT_RANGE;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_CHECKSUM) == 1
                warning('Checksum error!');
                errorCodes = errorCodes + obj.ERRBIT_CHECKSUM;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_OVERLOAD) == 1
                warning('Overload error!');
                errorCodes = errorCodes + obj.ERRBIT_OVERLOAD;
            end            
            if calllib('dynamixel','dxl_get_rxpacket_error',obj.ERRBIT_INSTRUCTION) == 1
                warning('Instruction code error!');
                errorCodes = errorCodes + obj.ERRBIT_INSTRUCTION;
            end
        end
        
        
        function printCommStatus(obj, commStatus)
        %MX64.printCommStatus() Prints out the communication status of the
        %  last TX/RX operation.  Also returns the status directly.
            
            switch(commStatus)
                case obj.COMM_TXFAIL
                    warning('COMM_TXFAIL : Failed to transmit instruction packet!');
                case obj.COMM_TXERROR
                    warning('COMM_TXERROR: Incorrect instruction packet!');
                case obj.COMM_RXFAIL
                    warning('COMM_RXFAIL: Failed to get status packet from device!');
                case obj.COMM_RXWAITING
                    warning('COMM_RXWAITING: Now recieving status packet!');
                case obj.COMM_RXTIMEOUT
                    warning('COMM_RXTIMEOUT: No status packet!');
                case obj.COMM_RXCORRUPT
                    warning('COMM_RXCORRUPT: Incorrect status packet!');
                otherwise
                    error('Unknown TX/RX Error!');
            end
        end
        
        function status = checkRXerror(obj)
        % MX64.checkRXerror()
        
            commStatus = calllib('dynamixel','dxl_get_result');
            if commStatus == obj.COMM_RXSUCCESS
                obj.printErrorCode(); % 0 = no error
            else
                obj.printCommStatus(commStatus);
            end
            status = commStatus;
        end
    end
end
