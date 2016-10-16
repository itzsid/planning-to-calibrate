globals
initialize_constants;
x = [.1:.2:1]*WORLD_SIZE;
            y = [.1:.2:1]*WORLD_SIZE;
            
            [X,Y] = meshgrid(x,y);

            figure(1); plot(X,Y, 'go'); xlim([0, 500]); ylim([0, 500]);
figure(2);
for i = 1:10
    x = [.1:.2:1]*WORLD_SIZE + 20*randn(1);
    y = [.1:.2:1]*WORLD_SIZE + 20*randn(1);            
    
    [X,Y] = meshgrid(x,y);
     plot(X,Y, 'go'); hold on; 
end
xlim([0, 500]); ylim([0, 500]);
    