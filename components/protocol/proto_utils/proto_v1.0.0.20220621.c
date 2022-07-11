#include "proto_v1.0.0.20220621.h"

update_pid_params_fun_t update_pid_params_fun_;

bool proto_register_update_pid_fun(update_pid_params_fun_t *update_pid_params_fun)
{
    update_pid_params_fun = update_pid_params_fun;
    return true;
}