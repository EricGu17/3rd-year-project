function[] = core_routine(self) % NOT YET IMPLEMENTED, DEMO STRUCTURE

    % demo code for going back and forth 5 times
    for i = 1:5 % change with while loop later
        
        dest_point = input('Which room? ', 's'); %Enter 'Red' to go to red room
        
        keyDest = {'Green','Blue','Red','Purple'};
        valueDest = [1 2 3 4];
        valueReturn = [2 1 4 3];
        
        Destination = containers.Map(keyDest,valueDest);
        Return = containers.Map(keyDest, valueReturn);
        
        %arm routine here to pick up medication
        %self.arm_routine("pick");
    
        %lfr routine here to go to room
        self.lfr_routine(Destination(dest_point));
        
        %arm routine here to put down medication
        %self.arm_routine("place");
        
        %180 turn at the end of forward pass
        flip(self);

        %lfr routine here to return to reference point
        self.lfr_routine(Return(dest_point));
        
        %180 turn turn at the end of backward pass
        flip(self);
    end
end
