#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <stack>
#include <algorithm>

struct XMLNode {
    int int_value;
    std::string value;  
    std::string file_name = "";
    std::unordered_map<std::string, std::vector<XMLNode>> children;

    std::vector<XMLNode>& operator[](const std::string& key) {
        return children[key];
    }

    XMLNode* find_first(const std::string& key) {
        auto it = children.find(key);
        if (it != children.end() && !it->second.empty()) {
            return &it->second.front(); 
        }

        for (auto& pair : children) {
            for (auto& child : pair.second) {
                XMLNode* result = child.find_first(key);
                if (result) return result; 
            }
        }

        return nullptr; 
    }

    void print(const std::string& tag = "root", int indent = 0) const {
        std::string indent_str(indent, ' ');

        // Stampa apertura tag con value se presente
        if (value.empty() && children.empty()) {
            std::cout << indent_str << "<" << tag << "/>\n";
        } else if (children.empty()) {
            std::cout << indent_str << "<" << tag << ">" << value << "</" << tag << ">\n";
        } else {
            std::cout << indent_str << "<" << tag << ">\n";
            for (const auto& child : children) {
                for (const auto& elem : child.second) {
                    elem.print(child.first, indent + 2);
                }
            }
            std::cout << indent_str << "</" << tag << ">\n";
        }
    }

    void setValue(const std::string& val) {
        value = val;
        try {
            int_value = std::stoi(val);
        }
        catch (const std::invalid_argument&) {
            int_value = 0;
        }
        catch (const std::out_of_range&) {
            int_value = 0;
        }
    }
};

class XMLParser {
private:
    std::string filename;

    std::string trim(const std::string& s) {
        size_t start = s.find_first_not_of(" \t\n\r");
        size_t end   = s.find_last_not_of(" \t\n\r");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }

public:
    XMLParser(const std::string& fname) : filename(fname) {}

    XMLNode parse() {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Errore: impossibile aprire " + filename);
        }

        XMLNode root;
        std::stack<XMLNode*> nodeStack;
        nodeStack.push(&root);
        size_t pos = filename.find_last_of("\\/");
        if (pos != std::string::npos) {
            root.file_name = filename.substr(pos + 1);
        } else {
            root.file_name = filename; 
        }


        std::string line;
        while (std::getline(file, line)) {
            line = trim(line);
            if (line.empty()) continue;

            if (line.substr(0, 2) == "<?") continue;  // salta dichiarazioni XML

            if (line[0] == '<' && line[1] != '/') {
                // tag apertura
                auto endTag = line.find('>');
                std::string tag = line.substr(1, endTag - 1);

                auto spacePos = tag.find(' ');
                if (spacePos != std::string::npos) {
                    tag = tag.substr(0, spacePos);
                }

                XMLNode newNode;
                nodeStack.top()->children[tag].push_back(newNode);

                if (line[endTag - 1] != '/') {
                    nodeStack.push(&nodeStack.top()->children[tag].back());
                }

                if (line.find("</") != std::string::npos) {
                    auto closeTag = line.find("</");
                    std::string value = line.substr(endTag + 1, closeTag - endTag - 1);
                    nodeStack.top()->setValue(trim(value));
                    nodeStack.pop();
                }
            } 
            else if (line.substr(0, 2) == "</") {
                nodeStack.pop();
            } 
            else {
                nodeStack.top()->setValue(line);
            }
        }

        file.close();
        return root;
    }
};


