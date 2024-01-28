#include "hwlist_agent.h"

u8 init_index = 0;

void hwlist_init(void) {
  /* initialize the hwlist pool */
  hwlist_init_node_pool();
}

void hwlist_require(struct Node **head, u32 (*child_if)(void),
                    u32 (*father_if)(void)) {

  struct node_data c_data;
  c_data.node_hw_if = child_if;
  struct node_data f_data;
  f_data.node_hw_if = father_if;

  // NOTE: check for existance of the father?
  // as long as a reference to a function i used it should be ok

  /* if father id is null, then add at the beginning of the LL */
  if (!father_if) {
    hwlist_insert_beginning(head, c_data);
  } else {
    /* if father is not null, add afer father node */

    struct Node *f_node_ptr;
    f_node_ptr = hwlist_search_frm_hwif(*head, f_data);
    hwlist_insert_after_node(f_node_ptr, c_data);
  }
  hwlist_agent_iterate(*head);
}

// TODO: move header
#define INIT_STATUS_NONINIT 0x00
#define INIT_STATUS_INITIALIZING 0x01
#define INIT_STATUS_INITIALIZED 0x03
void hwlist_agent_iterate(struct Node *node) {
  /*  Iterate from the last to the first node, executing the initialization
   * functions*/
  u32 init_output_status[HWLST_POOL_SIZE] = {0};

  /* a very inefficient way to do this, add a tail pointer? */
  while (node->next != NULL) {
    /* proceeds up to find the tail node */
    node = node->next;
  }

  /* initialization proceeds from tail to head */
  while (node != NULL) {
    if (!(node->init_status)) {
      node->init_status = INIT_STATUS_INITIALIZING;

      // struct node prev_node = node->prev_node;
      // node->init_order = prev_node.init_order + 1;
      node->init_order = node->next->init_order + 1;
      init_output_status[init_index] = node->data.node_hw_if();

      node->init_status = INIT_STATUS_INITIALIZED;

      char temp_str[50];
      int n;
      n = sprintf(
          temp_str,
          "initialized node | init_index %d = %x (fptr) | output: %d \r\n",
          init_index, node->data.node_hw_if, init_output_status[init_index]);
      uart_write_buf(USART2, temp_str, n);
      init_index++;
    }
    node = node->prev;
  }
}
