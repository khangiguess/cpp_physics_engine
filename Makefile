run:
	g++ main.cpp vector2D.cpp World.cpp RigidBody.cpp -o main -lsfml-graphics -lsfml-window -lsfml-system -std=c++17 && ./main