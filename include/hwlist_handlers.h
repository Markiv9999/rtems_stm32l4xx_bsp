#pragma once
/*
 * Defines the hardware linked list class
 * The objective is to have an hardware linked list
 *
 * when a system is used for the fist time:
 *  a node is registered and added at the end of the list.
 *  the dependancies for that hardware are placed before the last added node
 *  then the submodule initialization funcion will have access to
 *  the current hadware pointer to the node in the list.
 *  than it proceeds and initializes all the hardware
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "ext_typedefs.h"

#define HWLST_POOL_SIZE 30

/* define the node structure */
struct node_data {
  u32 (*node_hw_if)(void); // node hardware init function
};

struct Node {
  struct node_data data;
  bool in_use;
  bool initialized;
  struct Node *prev;
  struct Node *next;
};

void hwlist_init_node_pool(void);
struct Node *hwlist_get_newnode(struct node_data data);
void hwlist_insert_beginning(struct Node **head, struct node_data data);
void hwlist_insert_end(struct Node **head, struct node_data data);
void hwlist_insert_after_node(struct Node *prevNode, struct node_data data);
void hwlist_delete_frm_hwif(struct Node **head, struct node_data data);
struct Node *hwlist_search_frm_hwif(struct Node *head, struct node_data data);
void hwlist_print_list(struct Node *node);
