#include "ui_misc.h"

void label_text_list_t::set_text(std::string text) {
    if (head != NULL) {
        head->set_text(text);
    } else {
        head = new label_text_t(text, max_length);
    }
}

void label_text_list_t::add_text(std::string text) {
    if (head == NULL) {
        head = new label_text_t(text, max_length);
        return;
    }

    // Find the last node
    label_text_t *current = head;
    int count = 1; // Start with 1 for the head
    
    while (current->get_next() != NULL) {
        current = current->get_next();
        count++;
    }
    
    // Check if we've reached max length
    if (count >= max_length) {
        // Remove the oldest entry (the head)
        label_text_t *old_head = head;
        head = head->get_next();
        
        if (head != NULL) {
            head->set_prev(NULL);
        }
        
        delete old_head;
    }
    
    // Add the new entry at the end
    label_text_t *new_node = new label_text_t(text, max_length);
    current->set_next(new_node);
    new_node->set_prev(current);
}

void label_text_list_t::remove_text(std::string text) {
    if (head == NULL) {
        return;
    }
    
    label_text_t *current = head;
    
    // Check if the head contains the text
    if (current->get_text() == text) {
        head = current->get_next();
        
        if (head != NULL) {
            head->set_prev(NULL);
        }
        
        delete current;
        return;
    }
    
    // Check the rest of the list
    while (current->get_next() != NULL) {
        current = current->get_next();
        
        if (current->get_text() == text) {
            label_text_t *prev = current->get_prev();
            label_text_t *next = current->get_next();
            
            if (prev != NULL) {
                prev->set_next(next);
            }
            
            if (next != NULL) {
                next->set_prev(prev);
            }
            
            delete current;
            return;
        }
    }
}

void label_text_list_t::clear_text() {
    while (head != NULL) {
        label_text_t *temp = head;
        head = head->get_next();
        delete temp;
    }
}