#include <imgui.h>
#include <imgui-SFML.h>
#include <SFML/Graphics.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <unordered_set>
#include <vector>


// Utility stuff
// Hash for sf::Vectors2f's
struct Vector2HashFunction {
	size_t operator() (const sf::Vector2f& vec) const {
		size_t xHash = std::hash<float>() (vec.x);
		size_t yHash = std::hash<float>() (vec.y);
		return xHash ^ yHash;
	}
};
using VectorSet = std::unordered_set<sf::Vector2f, Vector2HashFunction>; // so its not a pain in the ass to type

// Returns the length of the vector
float length(const sf::Vector2f& vec) { return std::sqrtf(vec.x * vec.x + vec.y * vec.y); }
// Normalizes a vector to unit length
sf::Vector2f normalize(const sf::Vector2f& vec) { return vec / length(vec); }
// Returns a vector perpendicular to the given vector
sf::Vector2f perpendicular(const sf::Vector2f& vec) { return sf::Vector2f{ -vec.y, vec.x }; }
// Calculates the dot product between two vectors
float dot(const sf::Vector2f& left, const sf::Vector2f& right) { return (left.x * right.x) + (left.y * right.y); }


// Simple polygon class
class Polygon : public sf::Drawable, public sf::Transformable {
private:
	sf::VertexArray m_vertecies{ sf::LineStrip };
	sf::Vector2f m_velocity;
	float m_angularVelocity;
	sf::Color m_color;

private:
	// Renders the polygon onto the target
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
		states.transform *= getTransform();
		target.draw(m_vertecies, states);
	}

public:
	// Constructor
	Polygon(float radius, size_t numVertecies, const sf::Color color = { 255, 255, 255 })
		: m_velocity{ }, m_angularVelocity{ }, m_color{ color } {
		// NOTE: floating point error requires the use of an epsilon-like value of 0.01
		for (float theta = 0.0f; theta < 2.0f * std::numbers::pi - 0.01f; theta += 2.0f * std::numbers::pi / numVertecies) {
			m_vertecies.append({ radius * sf::Vector2f{ std::cosf(theta), -std::sinf(theta) }, m_color });
		}
		m_vertecies.append(m_vertecies[0]);
	}

	// Resets the polygon with a new radius and number of vertecies
	void reset(float radius, size_t numVertecies) {
		m_vertecies.clear();
		m_vertecies.resize(0);
		for (float theta = 0.0f; theta < 2.0f * std::numbers::pi - 0.01f; theta += 2.0f * std::numbers::pi / numVertecies) {
			m_vertecies.append({ radius * sf::Vector2f{ std::cosf(theta), -std::sinf(theta) }, m_color });
		}
		m_vertecies.append(m_vertecies[0]);
	}

	// Getters and setters
	float getRadius() const { return length(m_vertecies[0].position); }
	size_t getNumVertecies() const { return m_vertecies.getVertexCount() - 1; }
	sf::Vector2f getVelocity() const { return m_velocity; }
	void setVelocity(const sf::Vector2f& velocity) { m_velocity = velocity; }
	float getAngularVelocity() const { return m_angularVelocity; }
	void setAngularVelocity(float angularVelocity) { m_angularVelocity = angularVelocity; }
	void setColor(const sf::Color& color) {
		m_color = color;
		for (size_t i = 0; i < m_vertecies.getVertexCount(); ++i) {
			m_vertecies[i].color = color;
		}
	}

	// Returns true is the given point is within the polygon
	bool contains(const sf::Vector2f& point) {
		bool pointInsidePolygon = false;
		for (size_t i = 0; i < getNumVertecies(); ++i) {
			sf::Vector2f vc = getTransform().transformPoint(m_vertecies[i].position);
			sf::Vector2f vn = getTransform().transformPoint(m_vertecies[i + 1].position);

			if (((vc.y >= point.y && vn.y < point.y) || (vc.y < point.y && vn.y >= point.y)) &&
				(point.x < (vn.x - vc.x) * (point.y - vc.y) / (vn.y - vc.y) + vc.x)) pointInsidePolygon = !pointInsidePolygon;
		}

		return pointInsidePolygon;
	}

	// Returns the vertex and normal data of the polygon
	void getVertexData(VectorSet& normals, sf::VertexArray& vertecies) const {
		for (size_t i = 0; i < m_vertecies.getVertexCount() - 1; ++i) {
			sf::Vector2f vertex1 = getTransform().transformPoint(m_vertecies[i].position);
			sf::Vector2f vertex2 = getTransform().transformPoint(m_vertecies[i + 1].position);

			vertecies.append(vertex1);
			normals.insert(normalize(perpendicular(vertex2 - vertex1)));
		}
	}
};


// Uses the seperating axis theorem (SAT) to detect collisions
bool detectCollision(const Polygon* left, const Polygon* right, float& minPenetration, sf::Vector2f& penetrationAxis) {
	// Gets the vertecies and normals of the polygons
	VectorSet normals;
	sf::VertexArray leftVertecies, rightVertecies;
	left->getVertexData(normals, leftVertecies);
	right->getVertexData(normals, rightVertecies);

	// We need the minimum penetration
	minPenetration = std::numeric_limits<float>::max();

	// Iterates over each normal and checks for overlap
	for (const sf::Vector2f& normal : normals) {
		// Finds the minimum and maximum vertecies projected onto the normal vector
		float leftMin = std::numeric_limits<float>::max(), leftMax = -std::numeric_limits<float>::max();
		for (size_t i = 0; i < leftVertecies.getVertexCount(); ++i) {
			float projection = dot(leftVertecies[i].position, normal);
			if (projection < leftMin) leftMin = projection;
			if (projection > leftMax) leftMax = projection;
		}

		float rightMin = std::numeric_limits<float>::max(), rightMax = -std::numeric_limits<float>::max();
		for (size_t i = 0; i < rightVertecies.getVertexCount(); ++i) {
			float projection = dot(rightVertecies[i].position, normal);
			if (projection < rightMin) rightMin = projection;
			if (projection > rightMax) rightMax = projection;
		}

		// If there is overlap, calculate the penetration value and the normal
		if (std::max(leftMin, rightMin) <= std::min(leftMax, rightMax)) {
			float penetration = std::min(std::fabs(leftMax - rightMin), std::fabs(rightMax - leftMin));
			if (penetration < minPenetration) {
				minPenetration = penetration;
				penetrationAxis = (rightMin > leftMin) ? normal : normal * -1.0f;
			}
		} else return false;
	}

	return true;
}


// Main function
int main() {
	// Window
	sf::RenderWindow window{ sf::VideoMode{ 1200u, 675u }, "Collision Detection and Resolution using SAT", sf::Style::Close };
	ImGui::SFML::Init(window);
	sf::Clock clock;

	// Player data
	float playerRadius = 40.0f;
	int playerVertecies = 4;
	float maxSpeed = 100.0f;
	float maxAngularSpeed = 100.0f;

	// Polygons
	std::vector<Polygon*> polygons;
	Polygon* player = new Polygon{ playerRadius, static_cast<size_t>(playerVertecies), sf::Color::Red };
	player->setPosition(100.0f, 100.0f);
	polygons.push_back(player);
	polygons.push_back(new Polygon{ 50.0f, 4 });
	polygons.push_back(new Polygon{ 80.0f, 6 });
	polygons.at(1)->setPosition(400.0f, 200.0f);
	polygons.at(2)->setPosition(700.0f, 300.0f);
	polygons.at(1)->setAngularVelocity(100.0f);

	std::vector<std::pair<bool, sf::Vector2f>> polyDrag{ polygons.size(), std::make_pair(false, sf::Vector2f{ }) };

	// Main game loop
	while (window.isOpen()) {
		// Event loop
		sf::Event sfmlEvent;
		while (window.pollEvent(sfmlEvent)) {
			ImGui::SFML::ProcessEvent(sfmlEvent);

			switch (sfmlEvent.type) {
			// If the user closes the window
			case sf::Event::Closed:
				window.close();
				break;

			// If the user pressed a key
			case sf::Event::KeyPressed:
				if (sfmlEvent.key.code == sf::Keyboard::W) {
					player->setVelocity({ 0.0f, -maxSpeed });
				} if (sfmlEvent.key.code == sf::Keyboard::A) {
					player->setVelocity({ -maxSpeed, 0.0f });
				} if (sfmlEvent.key.code == sf::Keyboard::S) {
					player->setVelocity({ 0.0f,  maxSpeed });
				} if (sfmlEvent.key.code == sf::Keyboard::D) {
					player->setVelocity({  maxSpeed, 0.0f });
				} if (sfmlEvent.key.code == sf::Keyboard::Q) {
					player->setAngularVelocity(-maxAngularSpeed);
				} if (sfmlEvent.key.code == sf::Keyboard::E) {
					player->setAngularVelocity( maxAngularSpeed);
				} break;

			// If a key is released
			case sf::Event::KeyReleased:
				if (sfmlEvent.key.code == sf::Keyboard::W || sfmlEvent.key.code == sf::Keyboard::S) {
					player->setVelocity({ player->getVelocity().x, 0.0f });
				} if (sfmlEvent.key.code == sf::Keyboard::A || sfmlEvent.key.code == sf::Keyboard::D) {
					player->setVelocity({ 0.0f, player->getVelocity().y });
				} if (sfmlEvent.key.code == sf::Keyboard::Q || sfmlEvent.key.code == sf::Keyboard::E) {
					player->setAngularVelocity(0.0f);
				} break;

			// If the mouse button is pressed
			case sf::Event::MouseButtonPressed:
				if (sfmlEvent.mouseButton.button == sf::Mouse::Left) {
					sf::Vector2f mousePos = sf::Vector2f{ sf::Mouse::getPosition(window) };
					for (size_t i = 0; i < polygons.size(); ++i) {
						if (polygons.at(i)->contains(mousePos)) {
							polyDrag.at(i).first = true;
							polyDrag.at(i).second = mousePos - polygons.at(i)->getPosition();
						}
					}
				} break;

			// If the mouse button is pressed
			case sf::Event::MouseButtonReleased:
				if (sfmlEvent.mouseButton.button == sf::Mouse::Left) {
					for (auto& drag : polyDrag) drag.first = false;
				} break;
			}
		}

		// Even more input handling...
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
			sf::Vector2f mousePos = sf::Vector2f{ sf::Mouse::getPosition(window) };
			for (size_t i = 0; i < polygons.size(); ++i) {
				if (polyDrag.at(i).first) polygons.at(i)->setPosition(mousePos - polyDrag.at(i).second);
			}
		}

		// Update
		sf::Time clockRestart = clock.restart();
		float deltaTime = clockRestart.asSeconds();
		ImGui::SFML::Update(window, clockRestart);
		for (Polygon* polygon : polygons) {
			polygon->move(polygon->getVelocity() * deltaTime);
			polygon->rotate(polygon->getAngularVelocity() * deltaTime);
		}

		// Collision detection and resolution
		for (Polygon* left : polygons) {
			for (Polygon* right : polygons) {
				if (left == right) continue;
				float minPenetration;
				sf::Vector2f penetrationAxis;
				if (detectCollision(left, right, minPenetration, penetrationAxis)) {
					left->move(penetrationAxis * -minPenetration / 2.0f);
					right->move(penetrationAxis * minPenetration / 2.0f);
				}
			}
		}

		// GUI
		ImGui::Begin("Player Polygon");
		ImGui::SliderFloat("Radius", &playerRadius, 10.0f, 200.0f);
		ImGui::SliderInt("Number of vertecies", &playerVertecies, 0, 20);
		ImGui::SliderFloat("Speed", &maxSpeed, 10.0f, 300.0f);
		ImGui::SliderFloat("Angular Speed", &maxAngularSpeed, 10.0f, 300.0f);
		ImGui::End();
		if (player->getRadius() != playerRadius || player->getNumVertecies() != playerVertecies) {
			player->reset(playerRadius, static_cast<size_t>(playerVertecies));
		}

		// Render
		window.clear({ 50, 40, 80 });
		for (Polygon* polygon : polygons) {
			window.draw(*polygon);
		}

		// Display
		ImGui::SFML::Render(window);
		window.display();
	}

	ImGui::SFML::Shutdown();

	return 0;
}
