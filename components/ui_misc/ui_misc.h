#ifndef __UI_MISC_H__
#define __UI_MISC_H__

// #ifdef __cplusplus
// extern "C" {
// #endif

#include <string>

class label_text_t
{
    private:
    std::string text;
    label_text_t *next;
    label_text_t *prev;
    const int max_length;

    public:
    label_text_t(std::string text, int max_length = 5) : max_length(max_length)
    {
        this->text = text;
        this->next = NULL;
        this->prev = NULL;
    }

    void set_text(std::string text) {this->text = text;}
    void set_next(label_text_t *next) {this->next = next;}
    void set_prev(label_text_t *prev) {this->prev = prev;}

    std::string get_text() const {return this->text;}
    label_text_t *get_next() const {return this->next;}
    label_text_t *get_prev() const {return this->prev;}
};

class label_text_list_t
{
    private:
    label_text_t *head;
    const int max_length;

    public:
    label_text_list_t(int max_length = 5) : head(NULL), max_length(max_length) {}
    ~label_text_list_t() {clear_text();}

    std::string get_text() const {return this->head->get_text();}
    label_text_t *get_head() const {return this->head;}

    void set_text(std::string text);
    void add_text(std::string text);
    void remove_text(std::string text);
    void clear_text();
};


// #ifdef __cplusplus
// }
// #endif

#endif