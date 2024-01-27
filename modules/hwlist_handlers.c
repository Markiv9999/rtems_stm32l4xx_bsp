#include "hwlist_handlers.h"

/* define the node structure */
struct Node hwlist_node_pool[HWLST_POOL_SIZE];

/* initialize all nodes as not in use */
void hwlist_init_node_pool(void) {
  for (int i = 0; i < HWLST_POOL_SIZE; i++) {
    hwlist_node_pool[i].in_use = false;
    hwlist_node_pool[i].next = NULL;
  }
}

/*
 * - Finds the first node that is not in use in the init_node_pool
 * - Add information to the first free node, set as used
 * - Returns the pointer to the new node
 * */
struct Node *hwlist_get_newnode(struct node_data data) {
  for (int i = 0; i < HWLST_POOL_SIZE; i++) {
    if (!hwlist_node_pool[i].in_use) {
      hwlist_node_pool[i].data = data;
      hwlist_node_pool[i].in_use = true;
      hwlist_node_pool[i].next = NULL;
      hwlist_node_pool[i].prev = NULL;
      return &hwlist_node_pool[i];
    }
  }
  return NULL; // No available node
}

/*
 * fills the next node with data and gets its pointer
 * sets the next node from the new one as the old start of the list
 * sets the pointer to the head of the list to the new node
 */

void hwlist_insert_beginning(struct Node **head, struct node_data data) {
  struct Node *newNode = hwlist_get_newnode(data);
  if (newNode == NULL)
    return; // No available node

  newNode->next = *head;
  newNode->prev = NULL;

  if (*head != NULL) {
    (*head)->prev = newNode;
  }

  *head = newNode;
}

/*
 * fills the next node with data and gets its pointer
 * iterates up to reach the last element of the linked list
 * sets the pointer of the last element to the new node
 */

void hwlist_insert_end(struct Node **head, struct node_data data) {
  struct Node *newNode = hwlist_get_newnode(data);
  if (newNode == NULL)
    return; // No available node

  if (*head == NULL) {
    newNode->prev = NULL;
    *head = newNode;
    return;
  }

  struct Node *last = *head;
  while (last->next != NULL) {
    last = last->next;
  }

  last->next = newNode;
  newNode->prev = last;
}

/*
 * Insert node fater another node
 */
void hwlist_insert_after_node(struct Node *prevNode,
                              struct node_data input_data) {
  if (prevNode == NULL)
    return;

  struct Node *newNode = hwlist_get_newnode(input_data);
  if (newNode == NULL)
    return; // No available node

  newNode->next = prevNode->next;
  newNode->prev = prevNode;
  prevNode->next = newNode;

  if (newNode->next != NULL) {

    newNode->next->prev = newNode;
  }
}

void hwlist_delete_frm_hwif(struct Node **head, struct node_data input_data) {
  struct Node *temp = *head;

  while (temp != NULL && temp->data.node_hw_if != input_data.node_hw_if) {
    temp = temp->next;
  }

  if (temp == NULL)
    return; // Key not found

  if (temp->prev != NULL) {
    temp->prev->next = temp->next;

  } else {
    *head = temp->next;
  }

  if (temp->next != NULL) {
    temp->next->prev = temp->prev;
  }

  temp->in_use = false;
}

struct Node *hwlist_search_frm_hwif(struct Node *head,
                                    struct node_data input_data) {
  struct Node *current = head;
  while (current != NULL) {
    if (current->data.node_hw_if == input_data.node_hw_if)
      return current;
    current = current->next;
  }
  return NULL;
}

void hwlist_print_list(struct Node *node) {
  int i = 0;
  while (node != NULL) {
    char temp_str[50];
    int n;
    n = sprintf(temp_str, "node %d = %x \r\n", i, node->data.node_hw_if);
    uart_write_buf(USART2, temp_str, n);
    node = node->next;
    i++;
  }
  // printf("\n");
}
