#pragma once

// Stato cinematico del veicolo, aggiornato dalla pose_callback.
// Condiviso tra il nodo ROS e il VFHPlanner tramite riferimento const.
struct VehicleState
{
    double x{0.0},   y{0.0},   z{2.5};
    double pitch{0.0};
    double roll{0.0};
    double yaw{0.0}, yaw_rate{0.0};
    double vz{0.0};
};
