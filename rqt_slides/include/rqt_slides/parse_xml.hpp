#ifndef PARSEXML_H
#define PARSEXML_H

#include <tinyxml2/tinyxml2.h>
#include <string>
#include <memory>

namespace guionUtils
{

    class GuionParser
    {

    public:
        GuionParser(std::string path);
        ~GuionParser();

        bool loadXML(std::string path);
        std::string parseXML();

        tinyxml2::XMLElement *getSlide(int index);

        std::string getSlideHTMLContent(tinyxml2::XMLElement *slide);

        void elementToHtml(tinyxml2::XMLElement *element, std::string &htmlContent);

    private:
        tinyxml2::XMLDocument doc_;
    };
}

#endif // PARSEXML_H
