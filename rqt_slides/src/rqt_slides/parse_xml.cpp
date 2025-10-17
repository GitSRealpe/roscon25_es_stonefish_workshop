#include <rqt_slides/parse_xml.hpp>

#include <iostream>

namespace guionUtils
{

    GuionParser::GuionParser(std::string path)
    {

        tinyxml2::XMLError result = doc_.LoadFile(path.c_str());

        if (result != tinyxml2::XML_SUCCESS)
        {
            std::cerr << "Error parsing XML: " << doc_.ErrorStr() << std::endl;
        }

        // Get root element
        tinyxml2::XMLElement *guion = doc_.RootElement();
        if (!guion || strcmp(guion->Name(), "guion") != 0)
        {
            std::cerr << "Invalid root element, should be guion" << std::endl;
        }
    }

    GuionParser::~GuionParser()
    {
    }

    // bool XMLparser::loadXML(std::string path)
    // {

    //

    //     // Get slide element
    //     tinyxml2::XMLElement *slide = guion->FirstChildElement("slide");
    //     if (!slide)
    //     {
    //         std::cerr << "No slide element found" << std::endl;
    //     }

    //     // Get contenido element
    //     tinyxml2::XMLElement *contenido = slide->FirstChildElement("contenido");
    //     if (!contenido)
    //     {
    //         std::cerr << "No contenido element found" << std::endl;
    //     }

    //     // Get html element
    //     tinyxml2::XMLElement *html = contenido->FirstChildElement("html");
    //     if (!html)
    //     {
    //         std::cerr << "No html element found" << std::endl;
    //     }

    //     // Process all child elements of <html> recursively
    //     // for (tinyxml2::XMLElement *child = html->FirstChildElement(); child; child = child->NextSiblingElement())
    //     // {
    //     //   elementToHtml(child, htmlContent, 1);
    //     // }
    // }

    tinyxml2::XMLElement *GuionParser::getSlide(int index)
    {
        // Get root element
        tinyxml2::XMLElement *guionUtils = doc_.RootElement();
        if (!guionUtils || strcmp(guionUtils->Name(), "guion") != 0)
        {
            std::cerr << "Invalid root element" << std::endl;
            return nullptr;
        }

        // Find the slide element by index
        tinyxml2::XMLElement *slide = guionUtils->FirstChildElement("slide");
        int currentIndex = 0;
        while (slide && currentIndex < index)
        {
            slide = slide->NextSiblingElement("slide");
            currentIndex++;
        }

        if (!slide)
        {
            std::cerr << "Slide index " << index << " not found" << std::endl;
            return nullptr;
        }

        return slide;
    }

    std::string GuionParser::getSlideHTMLContent(tinyxml2::XMLElement *slide)
    {
        // Get contenido element
        tinyxml2::XMLElement *contenido = slide->FirstChildElement("contenido");
        if (!contenido)
        {
            std::cerr << "No contenido element found in slide " << std::endl;
            return "";
        }

        // Get html element
        tinyxml2::XMLElement *html = contenido->FirstChildElement("html");
        if (!html)
        {
            std::cerr << "No html element found in slide " << std::endl;
            return "";
        }

        std::string htmlContent = "<html><head/><body>";
        // Process all child elements of <html> recursively
        for (tinyxml2::XMLElement *child = html->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            elementToHtml(child, htmlContent);
        }

        htmlContent += "</body></html>";

        return htmlContent;
    }

    void GuionParser::elementToHtml(tinyxml2::XMLElement *element, std::string &htmlContent)
    {
        if (!element)
            return;

        // Get element name
        const char *tagName = element->Name();
        if (!tagName)
            return;

        // Open tag
        htmlContent += "<";
        htmlContent += tagName;

        // Add attributes if any
        for (const tinyxml2::XMLAttribute *attr = element->FirstAttribute(); attr; attr = attr->Next())
        {
            htmlContent += " " + std::string(attr->Name()) + "=\"" + attr->Value() + "\"";
        }
        htmlContent += ">";

        // Add text content if present
        const char *text = element->GetText();
        if (text)
        {
            htmlContent += std::string(text);
        }

        // Process child elements recursively
        bool hasChildren = false;
        for (tinyxml2::XMLElement *child = element->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            hasChildren = true;
            // htmlContent += "\n";
            elementToHtml(child, htmlContent);
        }

        // Close tag
        if (hasChildren)
        {
            // htmlContent += "\n";
        }
        htmlContent += "</" + std::string(tagName) + ">";
    }
}
