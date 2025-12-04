
export type TLifecycleTransition = { label: string; id: number };
/**
 * Lifecycle state of a node.
 */
export default class LifecycleState {
  /**
   * Unique identifier
   */
  id: string;

  /**
   * ROS name (including namespace)
   */
  name: string;

  state: string;

  available_transitions: TLifecycleTransition[] = [];

  constructor(
    id: string,
    name: string,
    state: string,
    available_transitions: TLifecycleTransition[],
  ) {
    this.id = id;
    this.name = name;
    this.state = state;
    this.available_transitions = available_transitions;
  }
}
