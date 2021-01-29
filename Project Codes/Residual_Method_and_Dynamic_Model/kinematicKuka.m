function [r1,r2,r3,r4,r5,r6,r7,a1,a2,a3,a4,a5,a6,a7,kWf7] = kinematicKuka (q, linkLens, ee_lenght);

    c1 = cos(q(1));
    s1 = sin(q(1));
    c2 = cos(q(2));
    s2 = sin(q(2));
    c3 = cos(q(3));
    s3 = sin(q(3));
    c4 = cos(q(4));
    s4 = sin(q(4));
    c5 = cos(q(5));
    s5 = sin(q(5));
    c6 = cos(q(6));
    s6 = sin(q(6));
    c7 = cos(q(7));
    s7 = sin(q(7));
    
    d1 = linkLens(2) + linkLens(1);
    d3 = linkLens(3) + linkLens(4);
    d5 = linkLens(5) + linkLens(6);
    d7 = linkLens(7) + ee_lenght;
   
    
    a1   =                  [c1 0  s1 0;
                             s1 0 -c1 0;
                             0  1  0  d1;
                             0  0  0  1];
    
    a2   =                  [c2 0  s2 0;
                             s2 0 -c2 0;
                             0  1  0  0;
                             0  0  0  1];
                       
    a3   =                  [c3 0  s3 0;
                             s3 0 -c3 0;
                             0  1  0  d3;
                             0  0  0  1];
                  
    a4   =                  [c4 0  s4 0;
                             s4 0 -c4 0;
                             0  1  0  0;
                             0  0  0  1];
                      
    a5   =                  [c5 0  s5 0;
                             s5 0 -c5 0;
                             0  1  0  d5;
                             0  0  0  1];
                         
    a6   =                  [c6 0  s6 0;
                             s6 0 -c6 0;
                             0  1  0  0;
                             0  0  0  1];
                         
    a7   =                  [c7  s7 0  0;
                             s7 -c7 0  0;
                             0   1  0  d7;
                             0   0  0  1];
                         
   
    
   
    kWf1 = a1;
    kWf2 = kWf1*a2;
    kWf3 = kWf2*a3;
    kWf4 = kWf3*a4;
    kWf5 = kWf4*a5;
    kWf6 = kWf5*a6;
    kWf7 = kWf6*a7;
    
    assignin('base','kWf7',kWf7);    
   
    r1 =                  [c1 0  s1;
                             s1 0 -c1;
                              0  1  0 ];
    
    r2 =                  [c2 0  s2;
                             s2 0 -c2;
                              0  1  0 ];
                       
    r3 =                  [c3 0  s3;
                             s3 0 -c3;
                              0  1  0 ];
                  
    r4 =                  [c4 0  s4;
                             s4 0 -c4;
                              0  1  0 ];
                      
    r5 =                  [c5 0  s5;
                             s5 0 -c5;
                              0  1  0 ];
                         
    r6 =                  [c6 0  s6;
                             s6 0 -c6;
                              0  1  0];
                         
    r7 =                  [c7  s7 0;
                             s7 -c7 0;
                             0   1  0 ];
                         
    assignin('base','r1',r1);
    assignin('base','r2',r2);
    assignin('base','r3',r3);
    assignin('base','r4',r4);
    assignin('base','r5',r5);
    assignin('base','r6',r6);
    assignin('base','r7',r7);
    
    rotWRTframe1 = r1   ;
    rotWRTframe2 = rotWRTframe1*r2;
    rotWRTframe3 = rotWRTframe2*r3;
    rotWRTframe4 = rotWRTframe3*r4;
    rotWRTframe5 = rotWRTframe4*r5;
    rotWRTframe6 = rotWRTframe5*r6;
    rotWRTframe7 = rotWRTframe6*r7;
    
    rWf1 = rotWRTframe1;
    rWf2 = rotWRTframe2;
    rWf3 = rotWRTframe3;
    rWf4 = rotWRTframe4;
    rWf5 = rotWRTframe5;
    rWf6 = rotWRTframe6;
    rWf7 = rotWRTframe7;
    
    assignin('base','rWf1',rWf1);
    assignin('base','rWf2',rWf2);
    assignin('base','rWf3',rWf3);
    assignin('base','rWf4',rWf4);
    assignin('base','rWf5',rWf5);
    assignin('base','rWf6',rWf6);
    assignin('base','rWf7',rWf7);
    
    
    
end

    
                       
                       