#include "hwlist_agent.h"

void hwlist_init(void) {
  /* initialize the hwlist pool */
  hwlist_init_node_pool();
}

void hwlist_require(struct Node **head, u32 (*child_if)(void),
                    void (*father_if)(void)) {

  struct node_data c_data;
  c_data.node_hw_if = child_if;
  struct node_data f_data;
  f_data.node_hw_if = child_if;

  /* if father id is null, then add at the beginning of the LL */
  if (!father_if) {
    hwlist_insert_beginning(head, c_data);
  } else {
    /* if father is not null, add afer father node */

    struct Node *f_node_ptr;
    f_node_ptr = hwlist_search_frm_hwif(*head, f_data);
    hwlist_insert_after_node(f_node_ptr, c_data);
  }
}

void hwlist_agent_iterate(struct Node *node) {
  while (node != NULL) {
    /* activate this initialization function */
    node->data.node_hw_if; // this is a function pointer. how can uoi call it?
    node = node->next;
    // the issue is that you need to go from the last to the first...
    // you should need a double linked list in this case..
    // can you do what you were tying to do if you are just able to go from low
    // to high?

    /* you can recreate a chain of addresses to have a backwards (execution)
     * address chain */
  }
}
