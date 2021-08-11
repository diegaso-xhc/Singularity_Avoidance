function get_trajectory(filename)


R_T = readtable('tele_sing3.csv');
T = table2array(R_T);
% Translations
Px = T(1:end - 1, 2);
Py = T(1:end - 1, 3);
Pz = T(1:end - 1, 4);
% Rotations (Euler Angles)
Rz = T(1:end - 1, 5);
Ry = T(1:end - 1, 6);
Rx = T(1:end - 1, 7);
% Euler
EulZYX = [Rz Ry Rx];
qZYX = [];
% dq_traj = [];

for i=1:size(EulZYX, 1)
    eul = EulZYX(i , :);
    qZYX(i,:) = eul2quat(eul,'ZYX');
end
p = [Px Py Pz];

for i=1:size(EulZYX,1)
    % convert the trans into DQ representation
    p_dq = p(i,1) * i_ + p(i,2) * j_ + p(i,3) * k_;
    % convert the orient
    r = qZYX(i,1) + qZYX(i,2) * i_ + qZYX(i,3) * j_ + qZYX(i,4) * k_;
    % pose in dq
    dq_traj = r + E_*0.5*p_dq*r;
    dq_vec(i,:) = vec8(dq_traj);
end
for i=1:size(EulZYX, 1)
    draw_vec= dq_vec(i,:);
    draw_dq = DQ([draw_vec]);
    G(:,:,i) = DQuaternionToMatrix(dq_vec(i,:));
    %        plot(p);
    %        grid on
    %        hold on
end
figure()
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
grid on
quiver3(G(1,4,:),G(2,4,:),G(3,4,:),G(1,1,:),G(1,2,:),G(1,3,:), 'color','r')
hold on
quiver3(G(1,4,:),G(2,4,:),G(3,4,:),G(2,1,:),G(2,2,:),G(2,3,:), 'color','g')
hold on
quiver3(G(1,4,:),G(2,4,:),G(3,4,:),G(3,1,:),G(3,2,:),G(3,3,:), 'color','b')
hold on
legend(' x',' y',' z')
end



