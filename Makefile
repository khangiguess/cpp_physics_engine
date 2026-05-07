run:
	g++ main.cpp vector2D.cpp World.cpp RigidBody.cpp -o main -lsfml-graphics -lsfml-window -lsfml-system -I/opt/homebrew/include -L/opt/homebrew/lib -std=c++17 && ./main
