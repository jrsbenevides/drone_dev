/*
 * droneMain.cpp
 *
 * Created by: rsinoue on 16.Jun.2017
 * Last Modified: jrsbenevides
 *
 * Description: This is the main function that forces the controller to run at X Hz, 
 *				where X is assigned in loop_rate(X).
 *
 *
 */

#include "drone/droneSystem.h"

int main(int argc, char **argv) // Argumentos em padrão: argc = número de parâmetros; **argv = endereço de cada parâmetro ?????
{
	// Initiate new ROS node named "recons_metric" motion_estimation_node
	ros::init(argc, argv, "motion_estimation");
//
	try {
		DRONE::System node; // Instancia objeto node????

		ros::Rate loop_rate(100); // Define a taxa de atualização???
		while (ros::ok())
		{
			node.control(); // Chama a função/método 
		    ros::spinOnce(); // Função para permitir executar o "node.control()";
		    loop_rate.sleep(); // Complementa o tempo restante para o período estipulado 10Hz
		}
		ros::spin();
	}
	catch (const std::exception &e) {
		ROS_FATAL_STREAM("An error has occurred: " << e.what());
		exit(1);
	}

  	return 0;
}
